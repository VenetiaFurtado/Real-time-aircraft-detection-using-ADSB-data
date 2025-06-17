# Dependancies:
```bash
sudo apt-get update
sudo apt-get install librtlsdr-dev libsfml-dev #to install rtlsdr
sudo apt install libsfml-dev
pkg-config --modversion sfml-all #to verify SFML installation
```
## Libraries and Resources Used in the Project

### Core Libraries:
- **RTL-SDR** (Software Defined Radio)  
  ↳ [GitHub: osmocom/rtl-sdr](https://github.com/osmocom/rtl-sdr)  
  *Used for SDR signal reception.*

- **dump1090** (ADS-B Decoder)  
  ↳ [GitHub: antirez/dump1090](https://github.com/antirez/dump1090)  
  *Decodes ADS-B signals from aircraft.*

- **acarsdec** (ACARS Decoder)  
  ↳ [GitHub: TLeconte/acarsdec](https://github.com/TLeconte/acarsdec)  
  *Decodes ACARS aircraft communication messages.*

- **SFML** (Graphics/Audio Library)  
  ↳ [SFML Linux Installation Guide](https://www.sfml-dev.org/tutorials/3.0/getting-started/linux/#installing-sfml)  
  *Used for visualization and UI rendering.*

### Map Data & Testing:
- **OpenStreetMap** (Map Tiles)  
  ↳ [Custom Map View](https://www.openstreetmap.org/#map=7/50.471/-124.102)  
  *Source for map tile rendering.*

- **Flightradar24** (Aircraft Tracking)  
  ↳ [Live Flight Tracking](https://www.flightradar24.com/49.04,-123.18/9)  
  *Used for testing/validation against real aircraft data.*
