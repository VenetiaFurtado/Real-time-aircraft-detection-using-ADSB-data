/*******************************************************************************
 * Author - Steve Rizor
 * Edited by - Venetia Furtado
 * Final Project:  Aircraft Detection using Automatic Dependent 
 * Surveillance–Broadcast (ADSB) Data
 * ECEN 5613 - Spring 2025
 * University of Colorado Boulder
 * Revised 04/29/2025
*
********************************************************************************
* This file initializes the system by setting up logging (openlog) and 
* creating circular buffers for ADS-B and ACARS data. It launches three threads -
* (readerThread) to continuously read radio signals (SDR) into these buffers and 
* two processing services are configured: processAcarsand 
* processAdsb, which handle message decoding. The function then enters an infinite 
* loop to visualize aircraft positions on a map (plotAircrafsOnMap).
*******************************************************************************/

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <string_view>
#include "Sequencer.hpp"
#include "adsb.h"

//#define BLOCK_SIZE (4 * 1024)
#define SET_BUFFER_LENGTH RTLOUTBUFSZ * 160 * 2

static Sequencer sequencer{};
static RtlSdr sdr{};
Adsb adsbObject{};
Acars acarsObject{};
Plotter plotter{};

//uint8_t bufferAdsb[SET_BUFFER_LENGTH];
uint8_t bufferAcars[SET_BUFFER_LENGTH];

CircularBuffer* adsbCb = nullptr;
CircularBuffer* acarsCb = nullptr;

void plotAircrafsOnMap()
{
    plotter.plotAircrafts(adsbObject.getAircrafts());
}

void processAdsb()
{
    RTLBuffer* b2 = adsbCb->getPtrToTail();
    if (b2 == nullptr || b2->n_read == 0)
    {
        return;
    }
   //printf("processAdsb: Read %d bytes from SDR\n", b2->n_read);
    adsbObject.processData(b2->buffer, b2->n_read);
    adsbCb->pop();
}

void processAcars()
{
    RTLBuffer* b2 = acarsCb->getPtrToTail();
    if (b2 == nullptr || b2->n_read == 0)
    {
        return;
    }

    acarsObject.processData(b2->buffer, b2->n_read);
    acarsCb->pop();
}

//Continuously reads data from an SDR (Software-Defined Radio) device and stores 
//it in two different buffers: one for ADSB and one for ACARS
void readBuffer()
{
    // For ADSB data
    RTLBuffer *bAdsb = adsbCb->getPtrToHead();
    if (bAdsb == nullptr)
    {
        return;
    }

    adsbCb->push();
    
    auto logTime = [](std::string_view log){auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
        syslog(LOG_INFO, "%s: %ld",log.data(), microseconds.count());
    };
    
    logTime("Before readSdr ADSB:");
    int readLen = sdr.readSdr(ADSB_FREQUENCY, bAdsb->buffer, BLOCK_SIZE);
    logTime("After readSdr ADSB:");

    if (readLen < 0)
    {
        perror("Unable to read from SDR");
        return;
    }

    // printf("readbuffer: read %d\n", readLen);

    bAdsb->n_read = readLen;

    // For ACARS data
    RTLBuffer *bAcars = acarsCb->getPtrToHead();
    if (bAcars == nullptr)
    {
        return;
    }

    acarsCb->push();

    readLen = sdr.readSdr(acarsObject.getFrequency(), bAcars->buffer, BLOCK_SIZE);

    if (readLen < 0)
    {
        perror("Unable to read from SDR");
        return;
    }

    // printf("readbuffer: read %d\n", readLen);

    bAcars->n_read = readLen;
}

#if 1
static void timerCallback(int sigid)
{
    trace(__func__);
#if 0
    auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count();
    syslog(LOG_INFO, "Starting service at time %ld ms",now_ms);
#endif
    sequencer.startServices();
}
#endif

static void cleanup(int sigid)
{
    sequencer.stopServices();
    sequencer.printStatistics();
    adsbObject.printAircrafts();
    closelog();
    plotter.close();
    delete adsbCb;
    delete acarsCb;
    cout << "\nExiting, bye!\n";
    exit(0);
}

int main()
{
    openlog("Sequencer", LOG_PID | LOG_CONS, LOG_USER);

    adsbCb = new CircularBuffer[CIRCULAR_BUFFER_SIZE];
    acarsCb = new CircularBuffer[CIRCULAR_BUFFER_SIZE];

    sequencer.addService(readBuffer, 2, 99, 300, "Reader thread");
    sequencer.addService(processAdsb, 1, 99, 140, "processAdsb");
    sequencer.addService(processAcars, 1, 98, 150, "processAcars");
    
    sequencer.initTimer(timerCallback);
    sequencer.startServices();

    std::signal(SIGINT, cleanup);
    std::cout << "Press Ctrl+C to terminate the program...\n";
    while (true)
    {
        // Infinite loop, waiting for Ctrl+C
        plotAircrafsOnMap();
    }
}
