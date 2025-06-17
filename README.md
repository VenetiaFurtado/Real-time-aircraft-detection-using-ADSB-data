# Project Overview

The **Aircraft Detection System** project leverages **Automatic Dependent Surveillance–Broadcast (ADS-B)**  
and **Aircraft Communications Addressing and Reporting System (ACARS)** to monitor and track aircraft in real-time.

This project aims to implement a **low-cost, reliable system** capable of receiving and decoding aircraft broadcast signals using **Software-Defined Radio (SDR)** and analyzing them through a combination of firmware and software components.  
The system is designed with **real-time processing considerations**, ensuring timely capture, decoding, and visualization of flight data for situational awareness.

---

## ADS-B: Automatic Dependent Surveillance–Broadcast

The ADS-B system, facilitated by tools like `dump1090`, enables the **reception and decoding of aircraft position data** broadcast over radio frequencies.

- `dump1090` is a widely used software decoder that:
  - Captures raw ADS-B signals received by an SDR dongle
  - Processes the data and converts it into readable formats

This pipeline allows **real-time tracking** of aircraft, providing details such as:

- Altitude
- Speed
- Flight path

These can be visualized on platforms like **FlightAware**, or integrated into **custom aviation applications**.

---

## ACARS: Aircraft Communications Addressing and Reporting System

ACARS is a **digital datalink system** used in aviation to **transmit short messages between aircraft and ground stations**.

### Key Features:
- Supports automated communication of:
  - Flight plans
  - Engine performance data
  - Maintenance alerts
  - Crew messages
- Operates via **VHF**, **HF**, or **satellite** links

Unlike ADS-B (which broadcasts real-time position), ACARS is **text-based** and improves **efficiency and safety** by reducing voice radio congestion.

### ACARS Decoder: `acarsdec`

- `acarsdec` is an open-source decoder specifically designed to:
  - **Demodulate and decode** ACARS messages
  - Process raw radio signals (typically from SDR dongles)
  - Extract structured ACARS data for analysis

---

> Together, ADS-B and ACARS provide complementary tools for monitoring aircraft behavior and communications, enabling a more complete picture of air traffic activity in real-time.
