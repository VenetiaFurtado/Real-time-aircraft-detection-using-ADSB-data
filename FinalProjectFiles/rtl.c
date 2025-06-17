/*
 *  Copyright (c) 2016 Thierry Leconte
 *
 *   
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License version 2
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifdef WITH_RTL

#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <rtl-sdr.h>
#include "acarsdec.h"
#include <signal.h>
#include <unistd.h>

// set the sameple rate by changing RTMULT
// 2.5Ms/s is the best but could be over limit for some hardware
// 2.0Ms/s is safer
// rtlMult 160	// 2.0000 Ms/s
// rtlMult 192	// 2.4000 Ms/s
// rtlMult 200   // 2.5000 Ms/s

#define RTLMULTMAX 320 // this is well beyond the rtl-sdr capabilities

static rtlsdr_dev_t *dev = NULL;
static int status = 0;
static int rtlInBufSize = 0;
static int rtlInRate = 0;

static int watchdogCounter = 50;
static pthread_mutex_t cbMutex = PTHREAD_MUTEX_INITIALIZER;

#define RTLOUTBUFSZ 1024


/* function verbose_device_search by Kyle Keen
 * from http://cgit.osmocom.org/rtl-sdr/tree/src/convenience/convenience.c
 */
int verbose_device_search(char *s)
{
	printf("verbose_device_search value of s: %c\n", *s);
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	if (verbose)
		fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (verbose)
			fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor,
				product, serial);
	}
	if (verbose)
		fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		if (verbose)
			fprintf(stderr, "Using device %d: %s\n",
				device,
				rtlsdr_get_device_name((uint32_t) device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;
		}
		device = i;
		if (verbose)
			fprintf(stderr, "Using device %d: %s\n",
				device,
				rtlsdr_get_device_name((uint32_t) device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;
		}
		device = i;
		if (verbose)
			fprintf(stderr, "Using device %d: %s\n",
				device,
				rtlsdr_get_device_name((uint32_t) device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;
		}
		if (strncmp(s, serial + offset, strlen(s)) != 0) {
			continue;
		}
		device = i;
		if (verbose)
			fprintf(stderr, "Using device %d: %s\n",
				device,
				rtlsdr_get_device_name((uint32_t) device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}

static unsigned int chooseFc(unsigned int *Fd, unsigned int nbch)
{
	int n;
	int ne;
	int Fc;
	do {
		ne = 0;
		for (n = 0; n < nbch - 1; n++) {
			if (Fd[n] > Fd[n + 1]) {
				unsigned int t;
				t = Fd[n + 1];
				Fd[n + 1] = Fd[n];
				Fd[n] = t;
				ne = 1;
			}
		}
	} while (ne);

	if ((Fd[nbch - 1] - Fd[0]) > rtlInRate - 4 * INTRATE) {
		fprintf(stderr, "Frequencies too far apart\n");
		return 0;
	}

	for (Fc = Fd[nbch - 1] + 2 * INTRATE; Fc > Fd[0] - 2 * INTRATE; Fc--) {
		for (n = 0; n < nbch; n++) {
			if (abs(Fc - Fd[n]) > rtlInRate / 2 - 2 * INTRATE)
				break;
			if (abs(Fc - Fd[n]) < 2 * INTRATE)
				break;
			if (n > 0 && Fc - Fd[n - 1] == Fd[n] - Fc)
				break;
		}
		if (n == nbch)
			break;
	}

	printf("ACARS Fc = %d\n", Fc);
	return Fc;
}

int nearest_gain(int target_gain)
{
	int i, err1, err2, count, close_gain;
	int *gains;
	count = rtlsdr_get_tuner_gains(dev, NULL);
	if (count <= 0) 
		return 0;
	gains = malloc(sizeof(int) * count);
	if(gains == NULL) 
		return 0;
	count = rtlsdr_get_tuner_gains(dev, gains);
	close_gain = gains[0];
	for (i = 0; i < count; i++) {
		err1 = abs(target_gain - close_gain);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			close_gain = gains[i];
		}
	}
	free(gains);
	return close_gain;
}

int initRtl(unsigned int* Fc_calculated)
{
	int n;
	unsigned int Fc;
	unsigned int Fd[MAXNBCHANNELS];

	printf("initRtl\n");

	if (rtlMult > RTLMULTMAX) 
	{
		fprintf(stderr, "rtlMult can't be larger than 360\n");
		return 1;
	}

    rtlInBufSize = RTLOUTBUFSZ * rtlMult * 2;
    rtlInRate = INTRATE * rtlMult;

	nbch = 0;
	float freq[3] = {131.475, 131.550, 131.725};
	while (nbch < 3 /*nbch < MAXNBCHANNELS*/) {
		Fd[nbch] =
		    ((int)(1000000 * freq[nbch] + INTRATE / 2) / INTRATE) *
		    INTRATE;
		channel[nbch].chn = nbch;
		channel[nbch].Fr = (float)Fd[nbch];
		nbch++;
	};
	if (nbch > MAXNBCHANNELS)
		fprintf(stderr,
			"WARNING: too many frequencies, using only the first %d\n",
			MAXNBCHANNELS);

	if (nbch == 0) {
		fprintf(stderr, "Need a least one frequency\n");
		return 1;
	}

	Fc = chooseFc(Fd, nbch); //finds the centre frequency of the three freqquencies up
	if (Fc == 0)
		return 1;
	*Fc_calculated = Fc;

	for (n = 0; n < nbch; n++) {
		channel_t *ch = &(channel[n]);
		int ind;
		float AMFreq;

		ch->wf = malloc(rtlMult * sizeof(float complex));
		ch->dm_buffer=malloc(RTLOUTBUFSZ*sizeof(float));
		if( ch->wf == NULL || ch->dm_buffer == NULL) {
			fprintf(stderr, "ERROR : malloc\n");
			return 1;
		}
		AMFreq = (ch->Fr - (float)Fc) / (float)(rtlInRate) * 2.0 * M_PI;
		for (ind = 0; ind < rtlMult; ind++) {
			ch->wf[ind]=cexpf(AMFreq*ind*-I)/rtlMult/127.5;
		}
	}

	if (verbose)
		fprintf(stderr, "Set center freq. to %dHz\n", (int)Fc);

	int res;
	for (n = 0; n < nbch; n++) {
		channel[n].chn = n;

		res = initMsk(&(channel[n]));
		if (res)
		{
			printf("initMsk failed!\n");
			break;
		}
		else
		{
			//printf("initMsk pass\n");
		}

		res = initAcars_serial(&(channel[n]));
		if (res)
		{
			printf("initAcars_serial failed!\n");
			break;
		}
		else
		{
			//printf("initAcars_serial pass\n");
		}

		initOutput();
	}

	return 0;
}

static void in_callback(unsigned char *rtlinbuff, uint32_t nread, void *ctx)
{
	int n;

	pthread_mutex_lock(&cbMutex);
	watchdogCounter = 50;
	pthread_mutex_unlock(&cbMutex);

	if (nread != rtlInBufSize) {
		fprintf(stderr, "warning: partial read\n");
		return;

	}
	status=0;

	// code requires this relationship set in initRtl:
	// rtlInBufSize = RTLOUTBUFSZ * rtlMult * 2;

	float complex vb[RTLMULTMAX];
	int i = 0;
	for (int m = 0; m < RTLOUTBUFSZ; m++) {
		for (int ind = 0; ind < rtlMult; ind++) {
			float r, g;

			r = (float)rtlinbuff[i] - 127.37f; i++;
			g = (float)rtlinbuff[i] - 127.37f; i++;

			vb[ind]=r+g*I;
		}

		for (n = 0; n < nbch; n++) {
			channel_t *ch = &(channel[n]);
			float complex D,*wf;

			wf = ch->wf;
			D = 0;
			for (int ind = 0; ind < rtlMult; ind++) {
				D += vb[ind] * wf[ind];
			}
			ch->dm_buffer[m]=cabsf(D);
		}
	}

	for (n = 0; n < nbch; n++) {
		channel_t *ch = &(channel[n]);
		demodMSK(ch,RTLOUTBUFSZ);
	}
}

static void processData(unsigned char *rtlinbuff, uint32_t nread)
{
	int n;

	if (nread != rtlInBufSize) {
		fprintf(stderr, "warning: partial read nread = %d rtlInBufSize  = %d\n ", nread, rtlInBufSize);
		return;

	}
	status=0;

	// code requires this relationship set in initRtl:
	// rtlInBufSize = RTLOUTBUFSZ * rtlMult * 2;

	float complex vb[RTLMULTMAX];
	int i = 0;
	for (int m = 0; m < RTLOUTBUFSZ; m++) {
		for (int ind = 0; ind < rtlMult; ind++) {
			float r, g;

			r = (float)rtlinbuff[i] - 127.37f; i++;
			g = (float)rtlinbuff[i] - 127.37f; i++;

			vb[ind]=r+g*I;
		}

		for (n = 0; n < nbch; n++) {
			channel_t *ch = &(channel[n]);
			float complex D,*wf;

			wf = ch->wf;
			D = 0;
			for (int ind = 0; ind < rtlMult; ind++) {
				D += vb[ind] * wf[ind];
			}
			ch->dm_buffer[m]=cabsf(D);
		}
	}

	for (n = 0; n < nbch; n++) {
		channel_t *ch = &(channel[n]);
		demodMSK_serial(ch,RTLOUTBUFSZ);
	}
}

static void *readThreadEntryPoint(void *arg) {
	rtlsdr_read_async(dev, in_callback, NULL, 4, rtlInBufSize);
	pthread_mutex_lock(&cbMutex);
	signalExit = 1;
	pthread_mutex_unlock(&cbMutex);
	return NULL;
}

int runRtlSample_serial(unsigned char* rtlinbuff, int n_read)
{
	//printf("rx data from sdr = len = %d\n", n_read);
	processData(rtlinbuff, n_read);
	blk_thread_serial();
	return 0;
}

int runRtlSample(void)
{
	pthread_t readThread;
	pthread_create(&readThread, NULL, readThreadEntryPoint, NULL);

	pthread_mutex_lock(&cbMutex);

	while (!signalExit) {
		if (--watchdogCounter <= 0) {
			fprintf(stderr, "No data from the SDR for 5 seconds, exiting ...\n");
			runRtlCancel(); // watchdog triggered after 5 seconds of no data from SDR
			break;
		}
		pthread_mutex_unlock(&cbMutex);
		usleep(100 * 1000); // 0.1 seconds
		pthread_mutex_lock(&cbMutex);
	}

	pthread_mutex_unlock(&cbMutex);

	int count = 100; // 10 seconds
	int err = 0;
	// Wait on reader thread exit
	while (count-- > 0 && (err = pthread_tryjoin_np(readThread, NULL))) {
		usleep(100 * 1000); // 0.1 seconds
	}
	if (err) {
		fprintf(stderr, "Receive thread termination failed, will raise SIGKILL to ensure we die!\n");
		raise(SIGKILL);
		return 1;
	}

	return 0;
}

int runRtlCancel(void) {
	if (dev) {
		rtlsdr_cancel_async(dev); // interrupt read_async
	}
	return 0;
}

int runRtlClose(void) {
	int res = 0;
	if (dev) {
		res = rtlsdr_close(dev);
		dev = NULL;
	}
	if (res) {
		fprintf(stderr, "rtlsdr_close: %d\n", res);
	}

	return res;
}
#endif
