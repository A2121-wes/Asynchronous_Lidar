[README.md](https://github.com/user-attachments/files/30673141/README.md)
# Asynchronous Lidar — Drone Implementation

A working hardware and software implementation of the asynchronous lidar system
described in:

> Glennie, C.L., Bui, L.K., Haces-Garcia, F., Lichti, D.D. (2025).
> *Asynchronous Lidar: Proof-of-concept simulation and demonstration tests.*
> ISPRS Open Journal of Photogrammetry and Remote Sensing, 17, 100096.
> https://doi.org/10.1016/j.ophoto.2025.100096

The paper demonstrated the concept in a laboratory. This repository extends that
work toward a **drone-mounted realization**, which the paper names as its next step.

---

## The concept in one paragraph

In conventional (monostatic) lidar, the laser transmitter and the detector sit on
the same platform and share a clock. In **asynchronous** lidar they are separated
onto different platforms. A transmitter drone fires laser pulses at the ground; one
or more receiver drones photograph the resulting spots. Because each platform carries
its own GNSS receiver, and GPS time is identical everywhere on Earth to within
nanoseconds, **no timing wire or radio link between the drones is required**. Each
platform independently timestamps its own data, and the records are matched by GPS
time afterward. Ground point coordinates are then recovered by non-linear least
squares from the intersecting transmit and receive vectors.

---

## System overview

```
        GPS satellites  (the shared clock — same time everywhere)
              │                                    │
              ▼                                    ▼
   ┌──────────────────────┐            ┌──────────────────────────┐
   │  TRANSMITTER         │            │  RECEIVER                │
   │  Arduino Uno R3      │            │  Raspberry Pi 3B         │
   │  Garmin GPS 18x LVC  │            │  NovAtel SPAN-IGM        │
   │  MPU6050 IMU         │            │  camera                  │
   │  520 nm green laser  │            │                          │
   │  → txlog.csv         │            │  → flight_log.csv        │
   └──────────────────────┘            └──────────────────────────┘
              │                                    │
              └──────────────┬─────────────────────┘
                             ▼
                   Laptop, after collection
              combine_logs.py  →  laptop_processor.py
                             ▼
                   3D ground point coordinates
```

No cable connects the two platforms. GPS time alone keeps them synchronised.

---

## Repository contents

### Transmitter (Arduino)

| File | Purpose |
|---|---|
| `transmitter_uno_garmin_FINAL.ino` | **Current version.** Fires the laser on each GPS second, logs position/attitude/time to SD. |
| `transmitter_uno_indoor_test.ino` | Indoor test without GPS — verifies IMU, SD, and laser using the Uno's own clock. |
| `gps_scanner.ino` | Diagnostic. Sweeps baud rates and serial polarities and prints raw output; used to identify the correct GPS settings. |

### Receiver (Raspberry Pi)

| File | Purpose |
|---|---|
| `novatel_recorder.py` | Parses NovAtel INSPVA messages, logs position and attitude, optionally captures images. |

### Post-processing (laptop)

| File | Purpose |
|---|---|
| `combine_logs.py` | Merges transmitter and receiver logs on GPS time. Handles the differing time formats, the leap-second offset, and multi-session log files. |
| `laptop_processor.py` | Detects laser spots in images, corrects for platform attitude, solves ground points. |
| `async_lidar_math.py` | Core mathematics — Equations 4–14 of the paper. Non-linear least squares solver and DOP computation. |
| `make_test_data.py` | Generates synthetic flight data for testing the pipeline without hardware. |

### Simulation (ROS2, validation only)

`transmitter_node.py`, `receiver_node.py`, `fusion_node.py`, `launch_async_lidar.py`,
`test_monte_carlo.py` — a ROS2 simulation used to validate the mathematics against
the paper's published results before hardware was built.

---

## Hardware

### Transmitter platform

| Component | Model | Notes |
|---|---|---|
| Microcontroller | Elegoo Uno R3 | ATmega328P |
| GNSS | Garmin GPS 18x LVC | See critical configuration note below |
| IMU | MPU6050 (GY-521) | I²C |
| Storage | microSD breakout (SPI) | See SD card note below |
| Laser switching | IRF520 MOSFET module | Isolates 12 V laser supply from the Uno |
| Laser | 520 nm green diode module | Same wavelength as the paper |
| Laser supply | 12 V DC | Connects only to the MOSFET, never to the Uno |

**Wiring**

```
Garmin GPS 18x LVC          MPU6050            microSD module
  Red    → 5 V                VCC → 5 V          VCC  → 5 V
  Black  → GND                GND → GND          GND  → GND
  White  → pin 4  (data)      SDA → A4           MOSI → pin 11
  Yellow → pin 5  (PPS)       SCL → A5           MISO → pin 12
  Green  → not used                              SCK  → pin 13
                                                 CS   → pin 10
IRF520 MOSFET module
  SIG → pin 7        V+ / V− screw terminals → 12 V supply
  VCC → 5 V          OUT screw terminals     → laser
  GND → GND
```

A breadboard is used to distribute 5 V and ground, as the Uno has insufficient
power pins for four peripherals.

### Receiver platform

| Component | Model | Notes |
|---|---|---|
| Computer | Raspberry Pi 3 Model B v1.2 | 64-bit Raspberry Pi OS |
| GNSS/INS | NovAtel SPAN-IGM | Tightly coupled GNSS/INS; supplies position *and* attitude |
| Antenna | GNSS antenna | To NovAtel |
| Camera | Arducam UC-350 (IMX219) | Not yet operational — see Known Issues |

The NovAtel connects via its AUX cable (P/N 01019015) to a Pi USB port and enumerates
as `/dev/ttyUSB0`–`ttyUSB2`. **`/dev/ttyUSB1` is the active command and data port.**

---

## Critical configuration notes

These two findings cost significant debugging time and are not obvious from the
documentation. They are recorded here so they need not be rediscovered.

### 1. The Garmin GPS 18x LVC uses inverted serial at 9600 baud

Two independent deviations from the expected configuration had to be corrected
together; fixing either alone still produces no usable data.

**Inverted polarity.** Garmin's technical specification states the unit
"transmits voltage levels from ground to the input voltage, **TIA-232-F (RS-232)
polarity**." RS-232 polarity is the logical inverse of the TTL polarity an Arduino
expects. The voltage range (0–5 V) is safe for the Uno, so nothing is damaged and
the wiring appears correct — but every bit is inverted and the NMEA parser receives
noise. The fix is SoftwareSerial's third argument:

```cpp
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN, true);  // true = inverted
```

**Non-default baud rate.** Garmin documents 4800 baud as the factory default for
the LVC. The unit used here runs at **9600 baud**, presumably reconfigured at some
point in its service life. Do not assume the documented default; verify with
`gps_scanner.ino`.

**Symptom if either is wrong:** the PPS pulse still fires and the laser still blinks,
but satellite count, position, and time remain zero indefinitely, even outdoors with
clear sky. The blinking laser makes the system appear functional when it is not.

### 2. This Garmin emits only `$GPRMC` sentences

`$GPRMC` carries position, time, date, speed, and course. It does **not** carry
satellite count or altitude — those live in `$GPGGA`, which this unit is not
configured to transmit.

Consequences:
- Satellite count is unavailable; use the RMC validity flag (`fix`) as the
  indicator of a good solution instead.
- **Altitude is unavailable from the transmitter GNSS.** The ground-point solution
  requires transmitter height, so this must currently be measured by other means.
  Enabling `$GPGGA` requires connecting the Garmin's green wire and issuing a
  `$PGRMO` command. *(Open item.)*

### 3. SD card compatibility

A 64 GB card failed to initialise; a 256 GB card works. Cards above 32 GB ship
formatted exFAT, which the Arduino SD library cannot read, and reformatting to
FAT32 on macOS requires Disk Utility's "MS-DOS (FAT)" option. If `SD.begin()`
fails, suspect the card format before the wiring.

### 4. Time scales differ between the two platforms

| Platform | Time representation |
|---|---|
| Garmin (transmitter) | UTC time-of-day `HH:MM:SS` plus date |
| NovAtel (receiver) | GPS week number × 604800 + seconds-of-week |

GPS time also runs **18 seconds ahead of UTC** (leap seconds accumulated through
2017). `combine_logs.py` converts both to a common scale and applies this offset.
Omitting the leap-second correction shifts everything by 18 seconds and produces
zero matches.

---

## Operating procedure

### Transmitter

1. Load `transmitter_uno_garmin_FINAL.ino` (requires the TinyGPSPlus library).
2. Place outdoors with unobstructed sky view.
3. Open the serial monitor at 115200 baud.
4. **Wait for `FIX OK` with plausible coordinates before beginning a run.**
   A `[no fix]` indication means the logged rows will contain zeros.
5. Data is written to `txlog.csv` on the SD card.

### Receiver

```bash
# Configure the NovAtel to stream, once per session
minicom -D /dev/ttyUSB1 -b 115200
#   inside minicom:
#     log inspvasa ontime 1
#     saveconfig
#   exit with Ctrl-A, X

# Record
python3 novatel_recorder.py \
    --receiver_id 0 \
    --port /dev/ttyUSB1 \
    --baud 115200 \
    --fps 1 \
    --output ~/flight_data \
    --no_camera

# Stop with Ctrl-C, then shut down cleanly
sudo shutdown -h now
```

`--fps 1` matches the transmitter's one-blink-per-second rate.

> **The two platforms must record during overlapping time windows.** They need not
> start together — GPS time handles the offset — but if their recording periods do
> not intersect there is nothing to match. Start the receiver first, then the
> transmitter, and run both for several minutes.

### Post-processing

```bash
python3 combine_logs.py --tx txlog.csv --rx flight_log.csv --out combined.csv
python3 laptop_processor.py --flight_dir ./flight_001 --output ./results
```

`laptop_processor.py` expects:

```
flight_001/
├── transmitter/
│   └── transmitter_log.csv
├── receiver_0/
│   ├── images/
│   └── flight_log.csv
├── receiver_1/   (optional)
└── receiver_2/   (optional)
```

Requires `opencv-python`, `numpy`, `pandas`, `matplotlib`.

---

## Validation

| Test | Result |
|---|---|
| Solver, noise-free input | Recovers the true ground point to 0.00 m |
| End-to-end pipeline, synthetic imagery | Recovers a known ground point to **1.2 mm** |
| Monte Carlo vs. paper Table 4 | Vertical uncertainties agree within ≈3 mm; both published trends reproduced (horizontal invariant with receiver count, vertical decreasing) |
| NovAtel INSPVA parser | Verified against manufacturer example messages, long and short forms |
| ROS2 simulation | 101 beams resolved, PDOP ≈ 1.6 |
| Log combiner | Correctly handles multi-session and mixed-format files; matches with 0.000 s residual on synthetic input |

A caveat on the Monte Carlo comparison: horizontal uncertainties run approximately
25 % above the paper's Table 4 at 500 m transmitter / 200 m receiver altitude, but
agree closely at 400 m. The paper does not state which altitude Table 4 corresponds
to, so this is most likely a difference in assumed geometry rather than an error in
the solver, which is exact on noise-free input.

---

## Status

**Working**

- Transmitter: GPS position and time, PPS-synchronised laser firing, IMU logging, SD storage
- Receiver: NovAtel position streaming and logging at 1 Hz
- Post-processing: mathematics, log combination, ground point solution (validated on synthetic data)

**Outstanding**

1. **Receiver camera.** The Arducam UC-350 is not detected by the Pi
   (`rpicam-hello --list-cameras` reports no cameras). Suspected ribbon-cable
   seating. Until resolved, no laser-spot imagery is available and the ground-point
   solution cannot be exercised on real data.
2. **Transmitter altitude.** Unavailable while the Garmin emits only `$GPRMC`.
3. **INS alignment.** The NovAtel reports `INS_ALIGNING` and returns zero attitude
   when stationary. SPAN units require motion to complete alignment; expected to
   resolve once platform-mounted.
4. **Simultaneous collection.** Both platforms have produced valid independent
   data, but not yet during an overlapping window. This is procedural rather than
   technical.
5. **Camera timing for flight.** Software-loop capture yields roughly 5 ms
   synchronisation (~2.5 cm at 5 m/s). Centimetre accuracy at flight speed will
   require a machine-vision camera accepting a hardware trigger from the NovAtel
   event output.

---

## Design notes

**Why the transmitter GNSS matters most.** The paper finds that horizontal ground
point uncertainty depends almost entirely on transmitter altitude and is nearly
independent of receiver altitude (Fig. 8), and that the optimal configuration
maximises the transmitter-to-receiver height ratio. Accordingly, the higher-grade
GNSS/INS is best allocated to the transmitter platform once a second unit is
available.

**Receiver count.** The paper concludes that three or four receivers balance cost
against precision, with only marginal improvement beyond four. The software
supports two through eight.

**Timing budget.** Ground point error from timing scales as
platform velocity × synchronisation error:

| Synchronisation method | Error at 5 m/s |
|---|---|
| Hardware trigger (NovAtel event output) | ≈ 0.05 cm |
| Software-loop capture (Pi) | ≈ 2.5 cm |
| Consumer action camera + flash sync | ≈ 16 cm |

---

## Acknowledgements

Implements the method of Glennie et al. (2025). The original work was supported in
part by the U.S. Army Corps of Engineers Cold Regions Research and Engineering
Laboratory (W913E5-20-C0003), the National Geospatial Intelligence Agency
(HM04762210003), and the National Science Foundation (Project 2324629).
