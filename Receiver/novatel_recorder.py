#!/usr/bin/env python3
"""
NovAtel + Arducam Recorder
==========================
Records camera images synced with NovAtel SPAN-IGM position + attitude.

Works for BOTH receiver drones (with camera) and the transmitter drone
(camera disabled with --no_camera).

HARDWARE:
  - NovAtel SPAN-IGM  → AUX port (Part #01019015) → Pi USB → /dev/ttyUSB0
  - Arducam UC-350 (IMX219) → Pi CSI ribbon cable
  - GNSS antenna → NovAtel

NOVATEL SETUP (send these commands to the NovAtel once via its config):
  unlogall
  log inspvasa ontime 0.05      # position + attitude at 20Hz
  saveconfig

INSPVA MESSAGE FORMAT:
  #INSPVAA,header;week,seconds,lat,lon,height,vN,vE,vU,roll,pitch,azimuth,status*crc
  Field  4 = latitude
  Field  5 = longitude
  Field  6 = height
  Field  9 = roll  (degrees)
  Field 10 = pitch (degrees)
  Field 11 = azimuth/yaw (degrees)

RUN ON RECEIVER PI:
  python3 novatel_recorder.py --receiver_id 0 --output /media/sdcard/flight

RUN ON TRANSMITTER PI:
  python3 novatel_recorder.py --transmitter --output /media/sdcard/flight

INSTALL:
  pip3 install pyserial picamera2
"""

import argparse
import csv
import math
import os
import time
import threading
from pathlib import Path

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("WARNING: pyserial not available — NovAtel will output zeros")

try:
    from picamera2 import Picamera2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    print("WARNING: picamera2 not available — no images")


# -----------------------------------------------------------------------
# NOVATEL READER — parses INSPVA messages over serial
# -----------------------------------------------------------------------

class NovAtelReader:
    """
    Reads INSPVA (or INSPVAS) messages from the NovAtel SPAN-IGM.

    The INSPVA message gives position + attitude already fused.
    We parse it continuously in a background thread and keep the
    latest values available via get().

    Message format (ASCII):
      #INSPVAA,COM1,0,31.0,FINESTEERING,1264,144088.000,...;
      WEEK,SECONDS,LAT,LON,HEIGHT,VN,VE,VU,ROLL,PITCH,AZIMUTH,STATUS*CRC

    The short form starts with %INSPVASA and has a smaller header.
    We handle both.
    """

    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.gps_week    = 0
        self.gps_seconds = 0.0
        self.lat         = 0.0
        self.lon         = 0.0
        self.height      = 0.0
        self.roll        = 0.0   # degrees
        self.pitch       = 0.0   # degrees
        self.azimuth     = 0.0   # degrees
        self.status      = 'NO_DATA'
        self.lock        = threading.Lock()
        self.connected   = False

        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baud, timeout=1.0)
                self.connected = True
                threading.Thread(target=self._run, daemon=True).start()
                print(f"NovAtel connected on {port} @ {baud} baud")
            except Exception as e:
                print(f"NovAtel connection error: {e}")
                print(f"  Check the port — try: ls /dev/ttyUSB*")
        else:
            print("NovAtel simulation mode")

    def _run(self):
        """Continuously read and parse INSPVA messages."""
        buffer = ''
        while True:
            try:
                data = self.ser.read(256).decode('ascii', errors='ignore')
                buffer += data

                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self._parse_line(line.strip())

            except Exception as e:
                print(f"NovAtel read error: {e}")
                time.sleep(0.1)

    def _parse_line(self, line):
        """Parse one INSPVA line into our fields."""
        if 'INSPVA' not in line:
            return

        try:
            # Split header from data at the semicolon
            if ';' not in line:
                return
            header, body = line.split(';', 1)

            # Remove checksum (everything after *)
            if '*' in body:
                body = body.split('*', 1)[0]

            fields = body.split(',')
            # fields: week, seconds, lat, lon, height, vN, vE, vU,
            #         roll, pitch, azimuth, status
            if len(fields) < 12:
                return

            with self.lock:
                self.gps_week    = int(float(fields[0]))
                self.gps_seconds = float(fields[1])
                self.lat         = float(fields[2])
                self.lon         = float(fields[3])
                self.height      = float(fields[4])
                self.roll        = float(fields[8])
                self.pitch       = float(fields[9])
                self.azimuth     = float(fields[10])
                self.status      = fields[11]

        except (ValueError, IndexError):
            return

    def get(self):
        with self.lock:
            return {
                'gps_week':    self.gps_week,
                'gps_seconds': self.gps_seconds,
                'lat':         self.lat,
                'lon':         self.lon,
                'height':      self.height,
                'roll':        self.roll,
                'pitch':       self.pitch,
                'azimuth':     self.azimuth,
                'status':      self.status,
            }

    def gps_time(self):
        """
        Convert GPS week + seconds to a single timestamp (seconds since
        GPS epoch 1980-01-06). This is the sync key across all drones.
        """
        with self.lock:
            return self.gps_week * 604800.0 + self.gps_seconds


def latlon_to_xyz(lat, lon, h, olat, olon, oh):
    R     = 6371000.0
    d_lat = math.radians(lat - olat)
    d_lon = math.radians(lon - olon)
    x     = R * d_lon * math.cos(math.radians(olat))
    y     = R * d_lat
    z     = h - oh
    return x, y, z


# -----------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--receiver_id', type=int, default=0)
    parser.add_argument('--transmitter', action='store_true',
                        help='This is the transmitter drone (no camera)')
    parser.add_argument('--no_camera',   action='store_true',
                        help='Disable camera even on a receiver')
    parser.add_argument('--output', type=str,
                        default=os.path.expanduser('~/flight_data'))
    parser.add_argument('--port',   type=str,   default='/dev/ttyUSB0')
    parser.add_argument('--baud',   type=int,   default=115200)
    parser.add_argument('--fps',    type=float, default=10.0)
    parser.add_argument('--width',  type=int,   default=1280)
    parser.add_argument('--height', type=int,   default=720)
    args = parser.parse_args()

    is_transmitter = args.transmitter
    use_camera     = CAMERA_AVAILABLE and not args.no_camera and not is_transmitter

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    if is_transmitter:
        (out_dir / 'drone_type.txt').write_text('transmitter')
        log_name = 'transmitter_log.csv'
        print("=== TRANSMITTER DRONE recorder ===")
    else:
        (out_dir / 'receiver_id.txt').write_text(str(args.receiver_id))
        log_name = 'flight_log.csv'
        print(f"=== RECEIVER {args.receiver_id} recorder ===")

    # Start NovAtel
    novatel = NovAtelReader(port=args.port, baud=args.baud)

    # Wait for NovAtel to get a good INS solution
    origin = None
    if novatel.connected:
        print("Waiting for NovAtel INS solution", end='', flush=True)
        for _ in range(120):
            d = novatel.get()
            if 'GOOD' in d['status'] or 'COMPLETE' in d['status']:
                origin = (d['lat'], d['lon'], d['height'])
                print(f"\nINS ready: {d['status']}")
                print(f"  Origin: ({d['lat']:.7f}, {d['lon']:.7f}, {d['height']:.2f}m)")
                break
            print('.', end='', flush=True)
            time.sleep(1.0)

    if origin is None:
        print("\nNo INS solution — using (0,0,0) origin")
        origin = (0.0, 0.0, 0.0)

    # Start camera (receivers only)
    cam = None
    img_dir = None
    if use_camera:
        img_dir = out_dir / 'images'
        img_dir.mkdir(exist_ok=True)
        cam = Picamera2()
        config = cam.create_still_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        print(f"Camera ready ({args.width}x{args.height})")

    # Open log
    log_file = open(out_dir / log_name, 'w', newline='')
    writer   = csv.writer(log_file)
    header = ['timestamp_gps', 'lat', 'lon', 'alt',
              'x_enu', 'y_enu', 'z_enu',
              'roll_deg', 'pitch_deg', 'yaw_deg',
              'roll_rad', 'pitch_rad', 'yaw_rad',
              'ins_status']
    if use_camera:
        header.append('image_file')
    writer.writerow(header)

    interval    = 1.0 / args.fps
    frame_count = 0

    print(f"\nRecording at {args.fps}fps — Ctrl+C to stop\n")
    print(f"  {'Frame':>6}  {'X':>8} {'Y':>8} {'Z':>8}  "
          f"{'Roll':>7} {'Pitch':>7} {'Yaw':>7}  {'Status':>20}")
    print("  " + "-" * 80)

    try:
        while True:
            t0 = time.time()

            d         = novatel.get()
            timestamp = novatel.gps_time() if novatel.connected else time.time()

            x, y, z = latlon_to_xyz(d['lat'], d['lon'], d['height'],
                                     origin[0], origin[1], origin[2])

            roll_deg  = d['roll']
            pitch_deg = d['pitch']
            yaw_deg   = d['azimuth']
            roll_rad  = math.radians(roll_deg)
            pitch_rad = math.radians(pitch_deg)
            yaw_rad   = math.radians(yaw_deg)

            row = [f"{timestamp:.6f}",
                   f"{d['lat']:.8f}", f"{d['lon']:.8f}", f"{d['height']:.3f}",
                   f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                   f"{roll_deg:.4f}", f"{pitch_deg:.4f}", f"{yaw_deg:.4f}",
                   f"{roll_rad:.6f}", f"{pitch_rad:.6f}", f"{yaw_rad:.6f}",
                   d['status']]

            if use_camera:
                img_file = f"{timestamp:.6f}.jpg"
                cam.capture_file(str(img_dir / img_file))
                row.append(img_file)

            writer.writerow(row)
            log_file.flush()

            frame_count += 1
            if frame_count % 10 == 0:
                print(f"  {frame_count:>6}  "
                      f"{x:>8.2f} {y:>8.2f} {z:>8.2f}  "
                      f"{roll_deg:>7.2f} {pitch_deg:>7.2f} {yaw_deg:>7.2f}  "
                      f"{d['status']:>20}")

            time.sleep(max(0, interval - (time.time() - t0)))

    except KeyboardInterrupt:
        print(f"\nStopped — {frame_count} records saved to {out_dir}")

    finally:
        log_file.close()
        if cam is not None:
            cam.stop()


if __name__ == '__main__':
    main()
