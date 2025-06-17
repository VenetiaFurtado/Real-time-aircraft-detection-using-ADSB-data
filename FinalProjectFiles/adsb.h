/*******************************************************************************
 * Source - This file has code imported from the following libraries:
 * class rtlsdr - Reference: https://github.com/librtlsdr/librtlsdr
 * class Plotter - https://www.sfml-dev.org/tutorials/3.0/getting-started/linux/#installing-sfml
 * class Adsb -  Reference: https://github.com/antirez/dump1090
 * class Acars - https://github.com/TLeconte/acarsdec
 * 
 * class CircularBuffer - was implemented for processAdsb and processAcars
 * Edited by - Venetia Furtado
 * Final Project:  Aircraft Detection using Automatic Dependent 
 * Surveillance–Broadcast (ADSB) Data
 * This code has been resued from previous Exercises of the course.
 * ECEN 5613 - Spring 2025
 * University of Colorado Boulder
 * Revised 04/29/2025
*
********************************************************************************/
#pragma once

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <math.h>
#include <complex>
#include <rtl-sdr.h>
#include <mutex>
#include <SFML/Graphics.hpp>  // for plotting
#include <unordered_map>
#include <syslog.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <atomic>
#include <stdexcept>

#define CIRCULAR_BUFFER_SIZE 100
#define BLOCK_SIZE RTLOUTBUFSZ * 160 * 2

extern "C"
{
#include "rtl.h"
}

#define MODES_AUTO_GAIN -100 /* Use automatic gain. */
#define MODES_DEFAULT_RATE 2000000
#define BUFFER_LENGTH (16 * 16384) /* 256k */
#define ADSB_FREQUENCY 1090e6
// #define ACARS_FREQUENCY 131.725e6

#define MODES_LONG_MSG_BITS 112  // ADSB message size without preamble
#define MODES_LONG_MSG_BYTES (112 / 8)

#define MODES_LONG_MSG_DATA_BITS 56 // ADSB data message size
#define MODES_SHORT_MSG_BITS 56

#define MODES_PREAMBLE_US 8   //microseconds
#define MODES_FULL_LEN (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS)

#define MODES_UNIT_FEET 0
#define MODES_UNIT_METERS 1

#define INTRATE 12500

//Set map bounds
#define TOP_LAT      49.5f
#define BOTTOM_LAT   49.0f
#define LEFT_LON     -123.3
#define RIGHT_LON    -122.5

/* ===================== Mode S detection and decoding  ===================== */
/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Structure used to describe an aircraft in iteractive mode. */
typedef struct
{
   uint32_t addr;   /* ICAO address */
   char hexaddr[7]; /* Printable ICAO address */
   char flight[9];  /* Flight number */
   int altitude;    /* Altitude */
   int speed;       /* Velocity computed from EW and NS components. */
   int track;       /* Angle of flight. */
   time_t seen;     /* Time at which the last packet was received. */
   long messages;   /* Number of Mode S messages received. */
   /* Encoded latitude and longitude as extracted by odd and even
    * CPR encoded messages. */
   int odd_cprlat;
   int odd_cprlon;
   int even_cprlat;
   int even_cprlon;
   double lat, lon; /* Coordinated obtained from CPR encoded data. */
   long long odd_cprtime, even_cprtime;
   struct aircraft *next; /* Next aircraft in our linked list. */

} Aircraft;

/*Interfaces with hardware and configures the antenna frequency*/
/******************************************************************************
 * Reference: https://github.com/librtlsdr/librtlsdr
******************************************************************************/
class RtlSdr
{
public:
   RtlSdr();
   int readSdr(const uint32_t frequency, uint8_t *buffer, uint32_t length);
   void closeSdr();

private:
   rtlsdr_dev_t *dev = nullptr;
   std::mutex rtlSdr_Mutex;
};

/* The struct we use to store information about a decoded message. */
/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
struct modesMessage
{
   /* Generic fields */
   unsigned char msg[MODES_LONG_MSG_BYTES]; /* Binary message. */
   int msgbits;                             /* Number of bits in message */
   int msgtype;                             /* Downlink format # */
   int crcok;                               /* True if CRC was valid */
   uint32_t crc;                            /* Message CRC */
   int errorbit;                            /* Bit corrected. -1 if no bit corrected. */
   int aa1, aa2, aa3;                       /* ICAO Address bytes 1 2 and 3 */
   int phase_corrected;                     /* True if phase correction was applied. */

   /* DF 11 */
   int ca; /* Responder capabilities. */

   /* DF 17 */
   int metype; /* Extended squitter message type. */
   int mesub;  /* Extended squitter message subtype. */
   int heading_is_valid;
   int heading;
   int aircraft_type;
   int fflag;            /* 1 = Odd, 0 = Even CPR message. */
   int tflag;            /* UTC synchronized? */
   int raw_latitude;     /* Non decoded latitude */
   int raw_longitude;    /* Non decoded longitude */
   char flight[9];       /* 8 chars flight number. */
   int ew_dir;           /* 0 = East, 1 = West. */
   int ew_velocity;      /* E/W velocity. */
   int ns_dir;           /* 0 = North, 1 = South. */
   int ns_velocity;      /* N/S velocity. */
   int vert_rate_source; /* Vertical rate source. */
   int vert_rate_sign;   /* Vertical rate sign. */
   int vert_rate;        /* Vertical rate. */
   int velocity;         /* Computed from EW and NS velocity. */

   /* DF4, DF5, DF20, DF21 */
   int fs;       /* Flight status for DF4,5,20,21 */
   int dr;       /* Request extraction of downlink request. */
   int um;       /* Request extraction of downlink request. */
   int identity; /* 13 bits identity (Squawk). */

   /* Fields used by multiple message types. */
   int altitude, unit;
};

//Displays aircrafts on a map using a graphical window
/******************************************************************************
 * Reference: https://www.sfml-dev.org/tutorials/3.0/getting-started/linux/#installing-sfml
******************************************************************************/
class Plotter
{
public:
   Plotter();
   void plotAircrafts(const std::unordered_map<uint32_t, Aircraft> &aircrafts);
   void close();
private:
   sf::Vector2f _latlonToPixel(float lat, float lon, int mapWidth, int mapHeight) //converts lat/lon position to pixels on the map
   {
      float x = (lon - LEFT_LON) / (RIGHT_LON - LEFT_LON) * mapWidth;
      float y = (TOP_LAT - lat) / (TOP_LAT - BOTTOM_LAT) * mapHeight;
      return sf::Vector2f(x, y);
   }

   sf::RenderWindow _window;
   sf::Texture _mapTex;
   sf::Texture _planeTex;
   sf::Sprite _mapSprite;
   bool _windowInit = false;
};

class Adsb
{
public:
/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
   Adsb();
   void processData(uint8_t *buffer, uint32_t length);
   void removeAircrafts();
   const std::unordered_map<uint32_t, Aircraft>& getAircrafts();
   void printAircrafts();
private:
/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
   /* Turn I/Q samples pointed in the buffer into the magnitude vector */
   void _computeMagnitudeVector(uint8_t *buffer, uint32_t length);

   /* Return -1 if the message is out of fase left-side
    * Return  1 if the message is out of fase right-size
    * Return  0 if the message is not particularly out of phase.
    *
    * Note: this function will access m[-1], so the caller should make sure to
    * call it only if we are not at the start of the current buffer. */
   int detectOutOfPhase(uint16_t *m);

   /* This function does not really correct the phase of the message, it just
    * applies a transformation to the first sample representing a given bit:
    *
    * If the previous bit was one, we amplify it a bit.
    * If the previous bit was zero, we decrease it a bit.
    *
    * This simple transformation makes the message a bit more likely to be
    * correctly decoded for out of phase messages:
    *
    * When messages are out of phase there is more uncertainty in
    * sequences of the same bit multiple times, since 11111 will be
    * transmitted as continuously altering magnitude (high, low, high, low...)
    *
    * However because the message is out of phase some part of the high
    * is mixed in the low part, so that it is hard to distinguish if it is
    * a zero or a one.
    *
    * However when the message is out of phase passing from 0 to 1 or from
    * 1 to 0 happens in a very recognizable way, for instance in the 0 -> 1
    * transition, magnitude goes low, high, high, low, and one of of the
    * two middle samples the high will be *very* high as part of the previous
    * or next high signal will be mixed there.
    *
    * Applying our simple transformation we make more likely if the current
    * bit is a zero, to detect another zero. Symmetrically if it is a one
    * it will be more likely to detect a one because of the transformation.
    * In this way similar levels will be interpreted more likely in the
    * correct way. */
   void applyPhaseCorrection(uint16_t *m);

   /* Given the Downlink Format (DF) of the message, return the message length
    * in bits. */
   int modesMessageLenByType(int type);

   uint32_t modesChecksum(unsigned char *msg, int bits);

   /* Decode the 12 bit AC altitude field (in DF 17 and others).
    * Returns the altitude or 0 if it can't be decoded. */
   int decodeAC12Field(unsigned char *msg, int *unit);

   /* Decode the 13 bit AC altitude field (in DF 20 and others).
    * Returns the altitude, and set 'unit' to either MODES_UNIT_METERS
    * or MDOES_UNIT_FEETS. */
   int decodeAC13Field(unsigned char *msg, int *unit);

   /* Always positive MOD operation, used for CPR decoding. */
   int cprModFunction(int a, int b);

   /* The NL function uses the precomputed table from 1090-WP-9-14 */
   int cprNLFunction(double lat);

   int cprNFunction(double lat, int isodd);

   double cprDlonFunction(double lat, int isodd);

   /* This algorithm comes from:
    * http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
    *
    *
    * A few remarks:
    * 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
    * 2) We assume that we always received the odd packet as last packet for
    *    simplicity. This may provide a position that is less fresh of a few
    *    seconds.
    */
   void decodeCPR(Aircraft *a);

   /* Decode a raw Mode S message demodulated as a stream of bytes by
    * detectModeS(), and split it into fields populating a modesMessage
    * structure. */
   void decodeModesMessage(struct modesMessage *mm, unsigned char *msg);

   void processModesMessage(struct modesMessage *mm);

   /* This function gets a decoded Mode S Message and prints it on the screen
    * in a human readable format. */
   void displayModesMessage(struct modesMessage *mm);

   /* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
    * size 'mlen' bytes. Every detected Mode S message is convert it into a
    * stream of bits and passed to the function to display it. */
   void detectModeS(uint32_t mlen);

   uint16_t _magnitudeVector[BUFFER_LENGTH];
   uint16_t _magnitudeLookupTable[129][129];
   std::unordered_map<uint32_t, Aircraft> _aircrafts;
};

class Acars
{
public:
/******************************************************************************
 * Reference: https://github.com/TLeconte/acarsdec
******************************************************************************/
   Acars();
   unsigned int getFrequency();
   void processData(uint8_t *buffer, uint32_t length);
   void logInit()
   {
      if(isLogInit == false)
      {
         openlog("Acars", LOG_PID | LOG_CONS, LOG_USER);
         isLogInit = true;
      }
   }
private:
   unsigned int frequency;
   bool isLogInit = false;
};

typedef struct
{
   uint8_t buffer[BLOCK_SIZE];
   int n_read;
} RTLBuffer;

class CircularBuffer
{
public:
   CircularBuffer()
       : _head(0), _tail(0), _size(0), _capacity(CIRCULAR_BUFFER_SIZE) {}

   RTLBuffer *getPtrToHead()
   {
      if (is_full())
      {
         //perror("Buffer is full");
         return nullptr;
      }
      RTLBuffer *res = &(_buffer[_head]);
      return res;
   }

   void push()
   {
      if (is_full())
      {
         //perror("Buffer is full");
         return;
      }
      _head = (_head + 1) % _capacity;
      _size++;
   }

   RTLBuffer *getPtrToTail()
   {
      if (is_empty())
      {
         //perror("Buffer is empty");
         return nullptr;
      }
      RTLBuffer *item = &(_buffer[_tail]);
      return item;
   }

   // Remove and return the oldest element
   void pop()
   {
      if (is_empty())
      {
         //perror("Buffer is empty");
         return;
      }
      _tail = (_tail + 1) % _capacity;
      _size--;
      return;
   }

   // Check if buffer is empty
   bool is_empty() { return _size == 0; }

   // Check if buffer is full
   bool is_full() { return _size == _capacity; }

   // Get current number of elements
   size_t size() { return _size; }

   // Get maximum capacity
   size_t capacity() { return _capacity; }

private:
   RTLBuffer _buffer[CIRCULAR_BUFFER_SIZE]; // Storage for elements
   size_t _head;                            // Index of oldest element
   size_t _tail;                            // Index where next element will be added
   std::atomic<size_t> _size;               // Current number of elements
   size_t _capacity;
};
