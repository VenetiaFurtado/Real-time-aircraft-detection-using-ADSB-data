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
#include "adsb.h"
#include <unordered_map>
#include <sys/time.h>
#include <stdio.h>

/*******************************************************************************
 * The following table has been imported from the dum1090 library:
 * https://github.com/antirez/dump1090
********************************************************************************/
/* Parity table for MODE S Messages.
 * The table contains 112 elements, every element corresponds to a bit set
 * in the message, starting from the first bit of actual data after the
 * preamble.
 *
 * For messages of 112 bit, the whole table is used.
 * For messages of 56 bits only the last 56 elements are used.
 *
 * The algorithm is as simple as xoring all the elements in this table
 * for which the corresponding bit on the message is set to 1.
 *
 * The latest 24 elements in this table are set to 0 as the checksum at the
 * end of the message should not affect the computation.
 *
 * Note: this function can be used with DF11 and DF17, other modes have
 * the CRC xored with the sender address as they are reply to interrogations,
 * but a casual listener can't split the address from the checksum.
 */
uint32_t modes_checksum_table[112] = {
   0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
   0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
   0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
   0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
   0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
   0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
   0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
   0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
   0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
   0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
   0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
   0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
   0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
   0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000};

/*******************************************************************************
 * The following function has been imported from the dump1090 library:
 * https://github.com/antirez/dump1090
********************************************************************************/
static long long mstime(void)
{
   struct timeval tv;
   long long mst;
   gettimeofday(&tv, NULL);
   mst = ((long long)tv.tv_sec) * 1000;
   mst += tv.tv_usec / 1000;
   return mst;
}

/*******************************************************************************
 * The following function has been referenced from the librtlsdr library:
 * https://github.com/librtlsdr/librtlsdr
********************************************************************************/
RtlSdr::RtlSdr()
{
   if (rtlsdr_open(&dev, 0) < 0)
   {
      perror("Error opening the RTLSDR device");
      exit(1);
   }

   int gains[100];
   // Setting to maximum gain for best performance
   int numgains = rtlsdr_get_tuner_gains(dev, gains);
   int maxgain = gains[numgains - 1];
   fprintf(stderr, "Max available gain is: %.2f\n", maxgain / 10.0);
   rtlsdr_set_tuner_gain(dev, maxgain);
   fprintf(stderr, "Set gain to: %.2f\n", rtlsdr_get_tuner_gain(dev) / 10.0);

   int ppm_error = 0;
   rtlsdr_set_freq_correction(dev, ppm_error);

   // Disable AGC since our signals from airplanes will not be very strong
   rtlsdr_set_agc_mode(dev, 0);

   // Set initial frequency even though we will change it in each service
   rtlsdr_set_center_freq(dev, ADSB_FREQUENCY);

   rtlsdr_set_sample_rate(dev, MODES_DEFAULT_RATE);

   rtlsdr_reset_buffer(dev);

   fprintf(stderr, "Gain reported by device: %.2f\n",
           rtlsdr_get_tuner_gain(dev) / 10.0);
}

/*******************************************************************************
 * Reference:https://github.com/librtlsdr/librtlsdr
********************************************************************************/
int RtlSdr::readSdr(const uint32_t frequency, uint8_t *buffer, uint32_t length)
{
   //std::lock_guard<std::mutex> lock(rtlSdr_Mutex);
   if (buffer == nullptr)
   {
      perror("buffer null\n");
   }

   rtlsdr_set_center_freq(dev, frequency);

   //how many bytes were actually read from hardware
   int n_read;

   // Blocking read
   if (rtlsdr_read_sync(dev, buffer, length, &n_read) < 0)
   {
      perror("SDR Read failed\n");
      return -1;
   }
   return n_read;
}

void RtlSdr::closeSdr()
{
   rtlsdr_close(dev);
}

Plotter::Plotter()
{
   _mapTex.loadFromFile("map.png");

   _mapSprite.setTexture(_mapTex);

   _planeTex.loadFromFile("red.png");
}

//Drawing aircraft on a map in real-time
/******************************************************************************
 * Reference: https://github.com/SFML/SFML
******************************************************************************/
void Plotter::plotAircrafts(const std::unordered_map<uint32_t, Aircraft> &aircrafts)
{
   if (_windowInit == false)
   {
      _window.create(sf::VideoMode(750, 800), "ADS-B Real-Time Plot");
      _windowInit = true;
   }

   if (_window.isOpen() == false)
   {
      return;
   }

   //  Render
   _window.clear();
   _window.draw(_mapSprite);
   for (auto &[_, a] : aircrafts)
   {
      if (a.lat == 0.0 || a.lon == 0.0) //Skips aircraft with invalid positions.
      {
         continue;
      }
      // printf("Plotting aircraft %d at lot = %f lon = %f\n\r", a.addr, a.lat, a.lon);
      sf::Vector2f pos = _latlonToPixel(a.lat, a.lon, _mapTex.getSize().x, _mapTex.getSize().y);
      sf::Sprite sprite;
      sprite.setTexture(_planeTex); //plane icon
      sprite.setScale(0.03f, 0.03f); // scaling - make icon small
      sprite.setPosition(pos); //paces icon at a specific point on map
      _window.draw(sprite);
   }
   _window.display();
}

void Plotter::close()
{
   _window.close();
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
Adsb::Adsb()
{
   /* Populate the I/Q -> Magnitude lookup table. It is used because
    * sqrt or round may be expensive and performance may vary a lot
    * depending on the libc used.
    *
    * Note that we don't need to fill the table for negative values, as
    * we square both i and q to take the magnitude. So the maximum absolute
    * value of i and q is 128, thus the maximum magnitude we get is:
    *
    * sqrt(128*128+128*128) = ~181.02
    *
    * Then, to retain the full resolution and be able to distinguish among
    * every pair of I/Q values, we scale this range from the float range
    * 0-181 to the uint16_t range of 0-65536 by multiplying for 360. */
   for (int i = 0; i <= 128; i++)
   {
      for (int q = 0; q <= 128; q++)
      {
         _magnitudeLookupTable[i][q] = round(sqrt(i * i + q * q) * 360);
      }
   }
}

void Adsb::processData(uint8_t *buffer, uint32_t length)
{
   _computeMagnitudeVector(buffer, length);
   detectModeS(length); //finds aircrafts
}

void Adsb::removeAircrafts()
{
   for (auto &[icao_addr, aircraft] : _aircrafts)
   {
      time_t start = time(nullptr); // get the current time
      time_t last_seen = aircraft.seen;
      if (difftime(last_seen, start) > 10)
      {
         // Remove the aircraft with key = icao_addr
         _aircrafts.erase(icao_addr);
      }
   }
}

const std::unordered_map<uint32_t, Aircraft> &Adsb::getAircrafts()
{
   return _aircrafts;
}

void Adsb::printAircrafts()
{
   for (const auto &[icao_addr, aircraft] : _aircrafts)
   {
      printf("X-----------------------------------------------------X\n");
      printf("    ICAO Addr : %x \n", icao_addr);
      printf("    Altitude : %d feet\n", aircraft.altitude);
      printf("    Latitude : %f \n", aircraft.lat);
      printf("    Longitude: %f \n", aircraft.lon);
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Turn I/Q samples pointed in the buffer into the magnitude vector */
void Adsb::_computeMagnitudeVector(uint8_t *buffer, uint32_t length)
{
   /* Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
    * we rescale to the 0-255 range to exploit the full resolution. */
   uint32_t magnitudeIter = 0;
   for (uint32_t j = 0; j < length; j += 2)
   {
      int i = buffer[j] - 127;
      int q = buffer[j + 1] - 127;

      if (i < 0)
      {
         i = -i;
      }
      if (q < 0)
      {
         q = -q;
      }

      _magnitudeVector[magnitudeIter++] = _magnitudeLookupTable[i][q];
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Return -1 if the message is out of fase left-side
 * Return  1 if the message is out of fase right-size
 * Return  0 if the message is not particularly out of phase.
 *
 * Note: this function will access m[-1], so the caller should make sure to
 * call it only if we are not at the start of the current buffer. */
int Adsb::detectOutOfPhase(uint16_t *m)
{
   if (m[3] > m[2] / 3)
      return 1;
   if (m[10] > m[9] / 3)
      return 1;
   if (m[6] > m[7] / 3)
      return -1;
   if (m[-1] > m[1] / 3)
      return -1;
   return 0;
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
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
void Adsb::applyPhaseCorrection(uint16_t *m)
{
   int j;

   m += 16; /* Skip preamble. */
   for (j = 0; j < (MODES_LONG_MSG_BITS - 1) * 2; j += 2)
   {
      if (m[j] > m[j + 1])
      {
         /* One */
         m[j + 2] = (m[j + 2] * 5) / 4;
      }
      else
      {
         /* Zero */
         m[j + 2] = (m[j + 2] * 4) / 5;
      }
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Given the Downlink Format (DF) of the message, return the message length
 * in bits. */
int Adsb::modesMessageLenByType(int type)
{
   if (type == 16 || type == 17 ||
       type == 19 || type == 20 ||
       type == 21)
      return MODES_LONG_MSG_BITS;
   else
      return MODES_SHORT_MSG_BITS;
}

uint32_t Adsb::modesChecksum(unsigned char *msg, int bits)
{
   uint32_t crc = 0;
   int offset = (bits == 112) ? 0 : (112 - 56);
   int j;

   for (j = 0; j < bits; j++)
   {
      int byte = j / 8;
      int bit = j % 8;
      int bitmask = 1 << (7 - bit);

      /* If bit is set, xor with corresponding table entry. */
      if (msg[byte] & bitmask)
         crc ^= modes_checksum_table[j + offset];
   }
   return crc; /* 24 bit checksum. */
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Decode the 12 bit AC altitude field (in DF 17 and others).
 * Returns the altitude or 0 if it can't be decoded. */
int Adsb::decodeAC12Field(unsigned char *msg, int *unit)
{
   int q_bit = msg[5] & 1;

   if (q_bit)
   {
      /* N is the 11 bit integer resulting from the removal of bit
       * Q */
      *unit = MODES_UNIT_FEET;
      int n = ((msg[5] >> 1) << 4) | ((msg[6] & 0xF0) >> 4);
      /* The final altitude is due to the resulting number multiplied
       * by 25, minus 1000. */
      return n * 25 - 1000;
   }
   else
   {
      return 0;
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Decode the 13 bit AC altitude field (in DF 20 and others).
 * Returns the altitude, and set 'unit' to either MODES_UNIT_METERS
 * or MDOES_UNIT_FEETS. */
int Adsb::decodeAC13Field(unsigned char *msg, int *unit)
{
   int m_bit = msg[3] & (1 << 6);
   int q_bit = msg[3] & (1 << 4);

   if (!m_bit)
   {
      *unit = MODES_UNIT_FEET;
      if (q_bit)
      {
         /* N is the 11 bit integer resulting from the removal of bit
          * Q and M */
         int n = ((msg[2] & 31) << 6) |
                 ((msg[3] & 0x80) >> 2) |
                 ((msg[3] & 0x20) >> 1) |
                 (msg[3] & 15);
         /* The final altitude is due to the resulting number multiplied
          * by 25, minus 1000. */
         return n * 25 - 1000;
      }
      else
      {
         /* TODO: Implement altitude where Q=0 and M=0 */
      }
   }
   else
   {
      *unit = MODES_UNIT_METERS;
      /* TODO: Implement altitude when meter unit is selected. */
   }
   return 0;
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Always positive MOD operation, used for CPR decoding. */
int Adsb::cprModFunction(int a, int b)
{
   int res = a % b;
   if (res < 0)
      res += b;
   return res;
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* The NL function uses the precomputed table from 1090-WP-9-14 */
int Adsb::cprNLFunction(double lat)
{
   if (lat < 0)
      lat = -lat; /* Table is simmetric about the equator. */
   if (lat < 10.47047130)
      return 59;
   if (lat < 14.82817437)
      return 58;
   if (lat < 18.18626357)
      return 57;
   if (lat < 21.02939493)
      return 56;
   if (lat < 23.54504487)
      return 55;
   if (lat < 25.82924707)
      return 54;
   if (lat < 27.93898710)
      return 53;
   if (lat < 29.91135686)
      return 52;
   if (lat < 31.77209708)
      return 51;
   if (lat < 33.53993436)
      return 50;
   if (lat < 35.22899598)
      return 49;
   if (lat < 36.85025108)
      return 48;
   if (lat < 38.41241892)
      return 47;
   if (lat < 39.92256684)
      return 46;
   if (lat < 41.38651832)
      return 45;
   if (lat < 42.80914012)
      return 44;
   if (lat < 44.19454951)
      return 43;
   if (lat < 45.54626723)
      return 42;
   if (lat < 46.86733252)
      return 41;
   if (lat < 48.16039128)
      return 40;
   if (lat < 49.42776439)
      return 39;
   if (lat < 50.67150166)
      return 38;
   if (lat < 51.89342469)
      return 37;
   if (lat < 53.09516153)
      return 36;
   if (lat < 54.27817472)
      return 35;
   if (lat < 55.44378444)
      return 34;
   if (lat < 56.59318756)
      return 33;
   if (lat < 57.72747354)
      return 32;
   if (lat < 58.84763776)
      return 31;
   if (lat < 59.95459277)
      return 30;
   if (lat < 61.04917774)
      return 29;
   if (lat < 62.13216659)
      return 28;
   if (lat < 63.20427479)
      return 27;
   if (lat < 64.26616523)
      return 26;
   if (lat < 65.31845310)
      return 25;
   if (lat < 66.36171008)
      return 24;
   if (lat < 67.39646774)
      return 23;
   if (lat < 68.42322022)
      return 22;
   if (lat < 69.44242631)
      return 21;
   if (lat < 70.45451075)
      return 20;
   if (lat < 71.45986473)
      return 19;
   if (lat < 72.45884545)
      return 18;
   if (lat < 73.45177442)
      return 17;
   if (lat < 74.43893416)
      return 16;
   if (lat < 75.42056257)
      return 15;
   if (lat < 76.39684391)
      return 14;
   if (lat < 77.36789461)
      return 13;
   if (lat < 78.33374083)
      return 12;
   if (lat < 79.29428225)
      return 11;
   if (lat < 80.24923213)
      return 10;
   if (lat < 81.19801349)
      return 9;
   if (lat < 82.13956981)
      return 8;
   if (lat < 83.07199445)
      return 7;
   if (lat < 83.99173563)
      return 6;
   if (lat < 84.89166191)
      return 5;
   if (lat < 85.75541621)
      return 4;
   if (lat < 86.53536998)
      return 3;
   if (lat < 87.00000000)
      return 2;
   else
      return 1;
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
int Adsb::cprNFunction(double lat, int isodd)
{
   int nl = cprNLFunction(lat) - isodd;
   if (nl < 1)
      nl = 1;
   return nl;
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
double Adsb::cprDlonFunction(double lat, int isodd)
{
   return 360.0 / cprNFunction(lat, isodd);
}
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
void Adsb::decodeCPR(Aircraft *a)
{
   const double AirDlat0 = 360.0 / 60;
   const double AirDlat1 = 360.0 / 59;
   double lat0 = a->even_cprlat;
   double lat1 = a->odd_cprlat;
   double lon0 = a->even_cprlon;
   double lon1 = a->odd_cprlon;

   /* Compute the Latitude Index "j" */
   int j = floor(((59 * lat0 - 60 * lat1) / 131072) + 0.5);
   double rlat0 = AirDlat0 * (cprModFunction(j, 60) + lat0 / 131072);
   double rlat1 = AirDlat1 * (cprModFunction(j, 59) + lat1 / 131072);

   if (rlat0 >= 270)
      rlat0 -= 360;
   if (rlat1 >= 270)
      rlat1 -= 360;

   /* Check that both are in the same latitude zone, or abort. */
   if (cprNLFunction(rlat0) != cprNLFunction(rlat1))
      return;

   /* Compute ni and the longitude index m */
   if (a->even_cprtime > a->odd_cprtime)
   {
      /* Use even packet. */
      int ni = cprNFunction(rlat0, 0);
      int m = floor((((lon0 * (cprNLFunction(rlat0) - 1)) -
                      (lon1 * cprNLFunction(rlat0))) /
                     131072) +
                    0.5);
      a->lon = cprDlonFunction(rlat0, 0) * (cprModFunction(m, ni) + lon0 / 131072);
      a->lat = rlat0;
   }
   else
   {
      /* Use odd packet. */
      int ni = cprNFunction(rlat1, 1);
      int m = floor((((lon0 * (cprNLFunction(rlat1) - 1)) -
                      (lon1 * cprNLFunction(rlat1))) /
                     131072.0) +
                    0.5);
      a->lon = cprDlonFunction(rlat1, 1) * (cprModFunction(m, ni) + lon1 / 131072);
      a->lat = rlat1;
   }
   if (a->lon > 180)
   {
      a->lon -= 360;
   }
}

/* Decode a raw Mode S message demodulated as a stream of bytes by
 * detectModeS(), and split it into fields populating a modesMessage
 * structure. */
void Adsb::decodeModesMessage(struct modesMessage *mm, unsigned char *msg)
{
   uint32_t crc2; /* Computed CRC, used to verify the message CRC. */
   const char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

   /* Work on our local copy */
   memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
   msg = mm->msg;

   /* Get the message type ASAP as other operations depend on this */
   mm->msgtype = msg[0] >> 3; /* Downlink Format */
   mm->msgbits = modesMessageLenByType(mm->msgtype);

   /* CRC is always the last three bytes. */
   mm->crc = ((uint32_t)msg[(mm->msgbits / 8) - 3] << 16) |
             ((uint32_t)msg[(mm->msgbits / 8) - 2] << 8) |
             (uint32_t)msg[(mm->msgbits / 8) - 1];
   crc2 = modesChecksum(msg, mm->msgbits);

   /* Check CRC and fix single bit errors using the CRC when
    * possible (DF 11 and 17). */
   mm->errorbit = -1; /* No error */
   mm->crcok = (mm->crc == crc2);

#if 0
      if (!mm->crcok && Modes.fix_errors &&
          (mm->msgtype == 11 || mm->msgtype == 17))
      {
         if ((mm->errorbit = fixSingleBitErrors(msg, mm->msgbits)) != -1)
         {
            mm->crc = modesChecksum(msg, mm->msgbits);
            mm->crcok = 1;
         }
         else if (Modes.aggressive && mm->msgtype == 17 &&
                  (mm->errorbit = fixTwoBitsErrors(msg, mm->msgbits)) != -1)
         {
            mm->crc = modesChecksum(msg, mm->msgbits);
            mm->crcok = 1;
         }
      }

#endif
   /* Note that most of the other computation happens *after* we fix
    * the single bit errors, otherwise we would need to recompute the
    * fields again. */
   mm->ca = msg[0] & 7; /* Responder capabilities. */

   /* ICAO address */
   mm->aa1 = msg[1];
   mm->aa2 = msg[2];
   mm->aa3 = msg[3];

   /* DF 17 type (assuming this is a DF17, otherwise not used) */
   mm->metype = msg[4] >> 3; /* Extended squitter message type. */
   mm->mesub = msg[4] & 7;   /* Extended squitter message subtype. */

   /* Fields for DF4,5,20,21 */
   mm->fs = msg[0] & 7;           /* Flight status for DF4,5,20,21 */
   mm->dr = msg[1] >> 3 & 31;     /* Request extraction of downlink request. */
   mm->um = ((msg[1] & 7) << 3) | /* Request extraction of downlink request. */
            msg[2] >> 5;

   /* In the squawk (identity) field bits are interleaved like that
    * (message bit 20 to bit 32):
    *
    * C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
    *
    * So every group of three bits A, B, C, D represent an integer
    * from 0 to 7.
    *
    * The actual meaning is just 4 octal numbers, but we convert it
    * into a base ten number tha happens to represent the four
    * octal numbers.
    *
    * For more info: http://en.wikipedia.org/wiki/Gillham_code */
   {
      int a, b, c, d;

      a = ((msg[3] & 0x80) >> 5) |
          ((msg[2] & 0x02) >> 0) |
          ((msg[2] & 0x08) >> 3);
      b = ((msg[3] & 0x02) << 1) |
          ((msg[3] & 0x08) >> 2) |
          ((msg[3] & 0x20) >> 5);
      c = ((msg[2] & 0x01) << 2) |
          ((msg[2] & 0x04) >> 1) |
          ((msg[2] & 0x10) >> 4);
      d = ((msg[3] & 0x01) << 2) |
          ((msg[3] & 0x04) >> 1) |
          ((msg[3] & 0x10) >> 4);
      mm->identity = a * 1000 + b * 100 + c * 10 + d;
   }

#if 0
      /* DF 11 & 17: try to populate our ICAO addresses whitelist.
       * DFs with an AP field (xored addr and crc), try to decode it. */
      if (mm->msgtype != 11 && mm->msgtype != 17)
      {
         /* Check if we can check the checksum for the Downlink Formats where
          * the checksum is xored with the aircraft ICAO address. We try to
          * brute force it using a list of recently seen aircraft addresses. */
         if (bruteForceAP(msg, mm))
         {
            /* We recovered the message, mark the checksum as valid. */
            mm->crcok = 1;
         }
         else
         {
            mm->crcok = 0;
         }
      }
      else
      {
         /* If this is DF 11 or DF 17 and the checksum was ok,
          * we can add this address to the list of recently seen
          * addresses. */
         if (mm->crcok && mm->errorbit == -1)
         {
            uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;
            addRecentlySeenICAOAddr(addr);
         }
      }
#endif

   /* Decode 13 bit altitude for DF0, DF4, DF16, DF20 */
   if (mm->msgtype == 0 || mm->msgtype == 4 ||
       mm->msgtype == 16 || mm->msgtype == 20)
   {
      mm->altitude = decodeAC13Field(msg, &mm->unit);
   }

   /* Decode extended squitter specific stuff. */
   if (mm->msgtype == 17)
   {
      /* Decode the extended squitter message. */

      if (mm->metype >= 1 && mm->metype <= 4)
      {
         /* Aircraft Identification and Category */
         mm->aircraft_type = mm->metype - 1;
         mm->flight[0] = ais_charset[msg[5] >> 2];
         mm->flight[1] = ais_charset[((msg[5] & 3) << 4) | (msg[6] >> 4)];
         mm->flight[2] = ais_charset[((msg[6] & 15) << 2) | (msg[7] >> 6)];
         mm->flight[3] = ais_charset[msg[7] & 63];
         mm->flight[4] = ais_charset[msg[8] >> 2];
         mm->flight[5] = ais_charset[((msg[8] & 3) << 4) | (msg[9] >> 4)];
         mm->flight[6] = ais_charset[((msg[9] & 15) << 2) | (msg[10] >> 6)];
         mm->flight[7] = ais_charset[msg[10] & 63];
         mm->flight[8] = '\0';
      }
      else if (mm->metype >= 9 && mm->metype <= 18)
      {
         /* Airborne position Message */
         mm->fflag = msg[6] & (1 << 2);
         mm->tflag = msg[6] & (1 << 3);
         mm->altitude = decodeAC12Field(msg, &mm->unit);
         mm->raw_latitude = ((msg[6] & 3) << 15) |
                            (msg[7] << 7) |
                            (msg[8] >> 1);
         mm->raw_longitude = ((msg[8] & 1) << 16) |
                             (msg[9] << 8) |
                             msg[10];
      }
      else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4)
      {
         /* Airborne Velocity Message */
         if (mm->mesub == 1 || mm->mesub == 2)
         {
            mm->ew_dir = (msg[5] & 4) >> 2;
            mm->ew_velocity = ((msg[5] & 3) << 8) | msg[6];
            mm->ns_dir = (msg[7] & 0x80) >> 7;
            mm->ns_velocity = ((msg[7] & 0x7f) << 3) | ((msg[8] & 0xe0) >> 5);
            mm->vert_rate_source = (msg[8] & 0x10) >> 4;
            mm->vert_rate_sign = (msg[8] & 0x8) >> 3;
            mm->vert_rate = ((msg[8] & 7) << 6) | ((msg[9] & 0xfc) >> 2);
            /* Compute velocity and angle from the two speed
             * components. */
            mm->velocity = sqrt(mm->ns_velocity * mm->ns_velocity +
                                mm->ew_velocity * mm->ew_velocity);
            if (mm->velocity)
            {
               int ewv = mm->ew_velocity;
               int nsv = mm->ns_velocity;
               double heading;

               if (mm->ew_dir)
                  ewv *= -1;
               if (mm->ns_dir)
                  nsv *= -1;
               heading = atan2(ewv, nsv);

               /* Convert to degrees. */
               mm->heading = heading * 360 / (M_PI * 2);
               /* We don't want negative values but a 0-360 scale. */
               if (mm->heading < 0)
                  mm->heading += 360;
            }
            else
            {
               mm->heading = 0;
            }
         }
         else if (mm->mesub == 3 || mm->mesub == 4)
         {
            mm->heading_is_valid = msg[5] & (1 << 2);
            mm->heading = (360.0 / 128) * (((msg[5] & 3) << 5) |
                                           (msg[6] >> 3));
         }
      }
   }
   mm->phase_corrected = 0; /* Set to 1 by the caller if needed. */
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
void Adsb::processModesMessage(struct modesMessage *mm)
{
   // return if CRC is not OK
   if (mm->crcok == false)
   {
      return;
   }

   if (mm->msgtype == 17)
   {
      uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;

      /* Decode the extended squitter message. */
      if (mm->metype >= 9 && mm->metype <= 18)
      {
         auto it = _aircrafts.find(addr);
         // if not found, create a new entry
         if (it == _aircrafts.end())
         {
            _aircrafts[addr] = Aircraft();
         }

         auto a = &(_aircrafts[addr]);

         a->seen = time(NULL);

         a->altitude = mm->altitude;

         if (mm->fflag != 0)
         {
            a->odd_cprlat = mm->raw_latitude;
            a->odd_cprlon = mm->raw_longitude;
            a->odd_cprtime = mstime();
         }
         else
         {
            a->even_cprlat = mm->raw_latitude;
            a->even_cprlon = mm->raw_longitude;
            a->even_cprtime = mstime();
         }

         /* If the two data is less than 10 seconds apart, compute
          * the position. */
         if (llabs(a->even_cprtime - a->odd_cprtime) <= 10000)
         {
            decodeCPR(a);
         }
      }
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* This function gets a decoded Mode S Message and prints it on the screen
 * in a human readable format. */
void Adsb::displayModesMessage(struct modesMessage *mm)
{
   int j;

   /* Show the raw message. */
   printf("*");
   for (j = 0; j < mm->msgbits / 8; j++)
      printf("%02x", mm->msg[j]);
   printf(";\n");

   printf("CRC: %06x (%s)\n", (int)mm->crc, mm->crcok ? "ok" : "wrong");

   if (mm->crcok == false)
   {
      printf("CRC not OK\n");
      return;
   }

   if (mm->errorbit != -1)
      printf("Single bit error fixed, bit %d\n", mm->errorbit);

   if (mm->msgtype == 0)
   {
      /* DF 0 */
      printf("DF 0: Short Air-Air Surveillance.\n");
      printf("  Altitude       : %d %s\n", mm->altitude,
             (mm->unit == MODES_UNIT_METERS) ? "meters" : "feet");
      printf("  ICAO Address   : %02x%02x%02x\n", mm->aa1, mm->aa2, mm->aa3);
   }
   else if (mm->msgtype == 4 || mm->msgtype == 20)
   {
      printf("DF %d: %s, Altitude Reply.\n", mm->msgtype,
             (mm->msgtype == 4) ? "Surveillance" : "Comm-B");
      // printf("  Flight Status  : %s\n", fs_str[mm->fs]);
      printf("  DR             : %d\n", mm->dr);
      printf("  UM             : %d\n", mm->um);
      printf("  Altitude       : %d %s\n", mm->altitude,
             (mm->unit == MODES_UNIT_METERS) ? "meters" : "feet");
      printf("  ICAO Address   : %02x%02x%02x\n", mm->aa1, mm->aa2, mm->aa3);

      if (mm->msgtype == 20)
      {
         /* TODO: 56 bits DF20 MB additional field. */
      }
   }
   else if (mm->msgtype == 5 || mm->msgtype == 21)
   {
      printf("DF %d: %s, Identity Reply.\n", mm->msgtype,
             (mm->msgtype == 5) ? "Surveillance" : "Comm-B");
      // printf("  Flight Status  : %s\n", fs_str[mm->fs]);
      printf("  DR             : %d\n", mm->dr);
      printf("  UM             : %d\n", mm->um);
      printf("  Squawk         : %d\n", mm->identity);
      printf("  ICAO Address   : %02x%02x%02x\n", mm->aa1, mm->aa2, mm->aa3);

      if (mm->msgtype == 21)
      {
         /* TODO: 56 bits DF21 MB additional field. */
      }
   }
   else if (mm->msgtype == 11)
   {
      /* DF 11 */
      printf("DF 11: All Call Reply.\n");
      // printf("  Capability  : %s\n", ca_str[mm->ca]);
      printf("  ICAO Address: %02x%02x%02x\n", mm->aa1, mm->aa2, mm->aa3);
   }
   else if (mm->msgtype == 17)
   {
      /* DF 17 */
      printf("DF 17: ADS-B message.\n");
      // printf("  Capability     : %d (%s)\n", mm->ca, ca_str[mm->ca]);
      printf("  ICAO Address   : %02x%02x%02x\n", mm->aa1, mm->aa2, mm->aa3);
      printf("  Extended Squitter  Type: %d\n", mm->metype);
      printf("  Extended Squitter  Sub : %d\n", mm->mesub);
      // printf("  Extended Squitter  Name: %s\n", getMEDescription(mm->metype,mm->mesub));

      /* Decode the extended squitter message. */
      if (mm->metype >= 1 && mm->metype <= 4)
      {
         /* Aircraft identification. */
         const char *ac_type_str[4] = {
             "Aircraft Type D",
             "Aircraft Type C",
             "Aircraft Type B",
             "Aircraft Type A"};

         printf("    Aircraft Type  : %s\n", ac_type_str[mm->aircraft_type]);
         printf("    Identification : %s\n", mm->flight);
      }
      else if (mm->metype >= 9 && mm->metype <= 18)
      {
         printf("    F flag   : %s\n", mm->fflag ? "odd" : "even");
         printf("    T flag   : %s\n", mm->tflag ? "UTC" : "non-UTC");
         printf("    Altitude : %d feet\n", mm->altitude);
         printf("    Latitude : %d (not decoded)\n", mm->raw_latitude);
         printf("    Longitude: %d (not decoded)\n", mm->raw_longitude);
      }
      else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4)
      {
         if (mm->mesub == 1 || mm->mesub == 2)
         {
            /* Velocity */
            printf("    EW direction      : %d\n", mm->ew_dir);
            printf("    EW velocity       : %d\n", mm->ew_velocity);
            printf("    NS direction      : %d\n", mm->ns_dir);
            printf("    NS velocity       : %d\n", mm->ns_velocity);
            printf("    Vertical rate src : %d\n", mm->vert_rate_source);
            printf("    Vertical rate sign: %d\n", mm->vert_rate_sign);
            printf("    Vertical rate     : %d\n", mm->vert_rate);
         }
         else if (mm->mesub == 3 || mm->mesub == 4)
         {
            printf("    Heading status: %d", mm->heading_is_valid);
            printf("    Heading: %d", mm->heading);
         }
      }
      else
      {
         printf("    Unrecognized ME type: %d subtype: %d\n",
                mm->metype, mm->mesub);
      }
   }
   else
   {
      printf("DF %d with good CRC received "
             "(decoding still not implemented).\n",
             mm->msgtype);
   }
}

/******************************************************************************
 * Reference: https://github.com/antirez/dump1090
******************************************************************************/
/* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
 * size 'mlen' bytes. Every detected Mode S message is convert it into a
 * stream of bits and passed to the function to display it. */
void Adsb::detectModeS(uint32_t mlen)
{
   uint16_t *m = _magnitudeVector;

   unsigned char bits[MODES_LONG_MSG_BITS];
   unsigned char msg[MODES_LONG_MSG_BITS / 2];
   uint16_t aux[MODES_LONG_MSG_BITS * 2];
   uint32_t j;
   int use_correction = 0;

   /******************************************************************************
   * Reference: https://github.com/antirez/dump1090
   ******************************************************************************/
   /* The Mode S preamble is made of impulses of 0.5 microseconds at
    * the following time offsets:
    *
    * 0   - 0.5 usec: first impulse.
    * 1.0 - 1.5 usec: second impulse.
    * 3.5 - 4   usec: third impulse.
    * 4.5 - 5   usec: last impulse.
    *
    * Since we are sampling at 2 Mhz every sample in our magnitude vector
    * is 0.5 usec, so the preamble will look like this, assuming there is
    * an impulse at offset 0 in the array:
    *
    * 0   -----------------
    * 1   -
    * 2   ------------------
    * 3   --
    * 4   -
    * 5   --
    * 6   -
    * 7   ------------------
    * 8   --
    * 9   -------------------
    */
   for (j = 0; j < mlen - MODES_FULL_LEN * 2; j++)
   {
      int low, high, delta, i, errors;
      int good_message = 0;

      if (use_correction)
         goto good_preamble; /* We already checked it. */

      /* First check of relations between the first 10 samples
       * representing a valid preamble. We don't even investigate further
       * if this simple test is not passed. */
      if (!(m[j] > m[j + 1] &&
            m[j + 1] < m[j + 2] &&
            m[j + 2] > m[j + 3] &&
            m[j + 3] < m[j] &&
            m[j + 4] < m[j] &&
            m[j + 5] < m[j] &&
            m[j + 6] < m[j] &&
            m[j + 7] > m[j + 8] &&
            m[j + 8] < m[j + 9] &&
            m[j + 9] > m[j + 6]))
      {
         // perror("Unexpected ratio among first 10 samples");
         continue;
      }

      /* The samples between the two spikes must be < than the average
       * of the high spikes level. We don't test bits too near to
       * the high levels as signals can be out of phase so part of the
       * energy can be in the near samples. */
      high = (m[j] + m[j + 2] + m[j + 7] + m[j + 9]) / 6;
      if (m[j + 4] >= high ||
          m[j + 5] >= high)
      {
         // perror("Too high level in samples between 3 and 6");
         continue;
      }

      /* Similarly samples in the range 11-14 must be low, as it is the
       * space between the preamble and real data. Again we don't test
       * bits too near to high levels, see above. */
      if (m[j + 11] >= high ||
          m[j + 12] >= high ||
          m[j + 13] >= high ||
          m[j + 14] >= high)
      {
         // perror("Too high level in samples between 10 and 15");
         continue;
      }

   good_preamble:
      /* If the previous attempt with this message failed, retry using
       * magnitude correction. */
      if (use_correction)
      {
         memcpy(aux, m + j + MODES_PREAMBLE_US * 2, sizeof(aux));
         if (j && detectOutOfPhase(m + j))
         {
            applyPhaseCorrection(m + j);
         }
         /* TODO ... apply other kind of corrections. */
      }

      /* Decode all the next 112 bits, regardless of the actual message
       * size. We'll check the actual message type later. */
      errors = 0;
      for (i = 0; i < MODES_LONG_MSG_BITS * 2; i += 2)
      {
         low = m[j + i + MODES_PREAMBLE_US * 2];
         high = m[j + i + MODES_PREAMBLE_US * 2 + 1];
         delta = low - high;
         if (delta < 0)
            delta = -delta;

         if (i > 0 && delta < 256)
         {
            bits[i / 2] = bits[i / 2 - 1];
         }
         else if (low == high)
         {
            /* Checking if two adiacent samples have the same magnitude
             * is an effective way to detect if it's just random noise
             * that was detected as a valid preamble. */
            bits[i / 2] = 2; /* error */
            if (i < MODES_SHORT_MSG_BITS * 2)
               errors++;
         }
         else if (low > high)
         {
            bits[i / 2] = 1;
         }
         else
         {
            /* (low < high) for exclusion  */
            bits[i / 2] = 0;
         }
      }

      /* Restore the original message if we used magnitude correction. */
      if (use_correction)
         memcpy(m + j + MODES_PREAMBLE_US * 2, aux, sizeof(aux));

      /* Pack bits into bytes */
      for (i = 0; i < MODES_LONG_MSG_BITS; i += 8)
      {
         msg[i / 8] =
             bits[i] << 7 |
             bits[i + 1] << 6 |
             bits[i + 2] << 5 |
             bits[i + 3] << 4 |
             bits[i + 4] << 3 |
             bits[i + 5] << 2 |
             bits[i + 6] << 1 |
             bits[i + 7];
      }

      int msgtype = msg[0] >> 3;
      int msglen = modesMessageLenByType(msgtype) / 8;

      /* Last check, high and low bits are different enough in magnitude
       * to mark this as real message and not just noise? */
      delta = 0;
      for (i = 0; i < msglen * 8 * 2; i += 2)
      {
         delta += abs(m[j + i + MODES_PREAMBLE_US * 2] -
                      m[j + i + MODES_PREAMBLE_US * 2 + 1]);
      }
      delta /= msglen * 4;

      /* Filter for an average delta of three is small enough to let almost
       * every kind of message to pass, but high enough to filter some
       * random noise. */
      if (delta < 10 * 255)
      {
         use_correction = 0;
         continue;
      }

      /* If we reached this point, and error is zero, we are very likely
       * with a Mode S message in our hands, but it may still be broken
       * and CRC may not be correct. This is handled by the next layer. */
      if (errors == 0)
      {
         // printf("Recevied message successfully\n");

         struct modesMessage mm;

         /* Decode the received message and update statistics */
         decodeModesMessage(&mm, msg);

         /* Pass data to the next layer */
         // useModesMessage(&mm);
         // displayModesMessage(&mm);
         processModesMessage(&mm);
      }
      else
      {
         // printf("The following message has %d demod errors\n", errors);
      }

      /* Retry with phase correction if possible. */
      if (!good_message && !use_correction)
      {
         j--;
         use_correction = 1;
      }
      else
      {
         use_correction = 0;
      }
   }
}

Acars::Acars()
{
   unsigned int *fptr = &frequency;
   initRtl(fptr);
}

unsigned int Acars::getFrequency()
{
   return frequency;
}

void Acars::processData(uint8_t *buffer, uint32_t length)
{
   //logInit();
   runRtlSample_serial(buffer, length);
}
