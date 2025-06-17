/*******************************************************************************
 * This file has been imported from the acarsdec library:
 * https://github.com/TLeconte/acarsdec
********************************************************************************
 *
 *  Copyright (c) 2015 Thierry Leconte
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <sched.h>
#include <unistd.h>
#include <limits.h>
#ifdef HAVE_LIBACARS
#include <libacars/version.h>
#endif
#include "acarsdec.h"
extern void build_label_filter(char *arg);

channel_t channel[MAXNBCHANNELS];
unsigned int nbch;

char *idstation = NULL;
int inmode = 0;
int verbose = 1;
int outtype = OUTTYPE_STD;
int netout = NETLOG_NONE;
int airflt = 0;
int emptymsg = 0;
int mdly=600;
int hourly = 0;
int daily = 0;

int signalExit = 0;

#ifdef HAVE_LIBACARS
int skip_reassembly = 0;
#endif

#ifdef WITH_RTL
int gain = -100;
int ppm = 0;
int rtlMult = 160;
#endif

#ifdef WITH_AIR
int gain = 18;
#endif

#ifdef	WITH_SDRPLAY
int	lnaState	= 2;
int	GRdB		= 20;
int	ppm		= 0;
#endif
#ifdef WITH_SOAPY
char *antenna=NULL;
double gain = -10.0;
int ppm = 0;
int rateMult = 160;
int freq = 0;
#endif

#ifdef WITH_MQTT
char *mqtt_urls[16];
int mqtt_nburls=0;
char *mqtt_topic=NULL;
char *mqtt_user=NULL;
char *mqtt_passwd=NULL;
#endif

char *Rawaddr = NULL;
char *logfilename = NULL;
