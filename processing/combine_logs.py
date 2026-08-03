#!/usr/bin/env python3
"""
Combine Transmitter + Receiver Logs into one 

RUN:
  python3 combine_logs.py --tx txlog.csv --rx flight_log.csv --out combined.csv
"""

import argparse
import csv
from datetime import datetime, timezone, timedelta

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)


def utc_to_gps_seconds(date_str, time_str, leap_seconds=18):
    dt = datetime.strptime(f"{date_str} {time_str}", "%Y-%m-%d %H:%M:%S")
    dt = dt.replace(tzinfo=timezone.utc)
    return (dt - GPS_EPOCH).total_seconds() + leap_seconds


def gps_seconds_to_readable(gps_sec, leap_seconds=18):
    dt = GPS_EPOCH + timedelta(seconds=gps_sec - leap_seconds)
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def load_transmitter(path, leap_seconds=18, verbose=True):
    """
    Read the transmitter log line by line, tracking header changes.
    The Uno appends a new header every time it restarts, and older
    sessions may use a different column layout. We follow along and
    keep only rows that have a real date, a real time, and fix=1.
    """
    rows = []
    header = None
    sessions = 0
    skipped_nofix = 0
    skipped_oldfmt = 0

    with open(path, newline='') as f:
        for raw in csv.reader(f):
            if not raw:
                continue

            # Is this a header row? (first cell is the literal column name)
            if raw[0].strip() == 'blink_number':
                header = [h.strip() for h in raw]
                sessions += 1
                continue

            if header is None:
                continue

            r = dict(zip(header, raw))

            # Old-format sessions have no gps_date column at all
            if 'gps_date' not in r:
                skipped_oldfmt += 1
                continue

            # Skip rows with no GPS fix
            if r.get('fix', '0').strip() != '1':
                skipped_nofix += 1
                continue

            date_str = r.get('gps_date', '').strip()
            time_str = r.get('gps_time', '').strip()
            if not date_str or date_str.startswith('0000') or time_str == '00:00:00':
                skipped_nofix += 1
                continue

            try:
                r['gps_seconds'] = utc_to_gps_seconds(date_str, time_str, leap_seconds)
            except ValueError:
                continue

            rows.append(r)

    if verbose:
        print(f"  sessions found in file:     {sessions}")
        print(f"  rows skipped (old format):  {skipped_oldfmt}")
        print(f"  rows skipped (no GPS fix):  {skipped_nofix}")
        print(f"  rows KEPT (valid fix):      {len(rows)}")

    rows.sort(key=lambda x: x['gps_seconds'])
    return rows


def load_receiver(path, verbose=True):
    """Read the NovAtel log, also tolerating repeated headers."""
    rows = []
    header = None
    with open(path, newline='') as f:
        for raw in csv.reader(f):
            if not raw:
                continue
            if raw[0].strip() == 'timestamp_gps':
                header = [h.strip() for h in raw]
                continue
            if header is None:
                continue
            r = dict(zip(header, raw))
            try:
                r['gps_seconds'] = float(r['timestamp_gps'])
            except (ValueError, KeyError):
                continue
            if r['gps_seconds'] <= 0:
                continue
            rows.append(r)
    if verbose:
        print(f"  rows loaded: {len(rows)}")
    rows.sort(key=lambda x: x['gps_seconds'])
    return rows


def match(tx_rows, rx_rows, tolerance=0.5):
    """Pair each transmitter blink with the nearest receiver record in time."""
    matched = []
    if not rx_rows:
        return matched
    rx_times = [r['gps_seconds'] for r in rx_rows]

    for tx in tx_rows:
        t = tx['gps_seconds']
        lo, hi = 0, len(rx_times) - 1
        while lo < hi:                      # binary search for nearest
            mid = (lo + hi) // 2
            if rx_times[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        best_i, best_d = lo, abs(rx_times[lo] - t)
        if lo > 0 and abs(rx_times[lo - 1] - t) < best_d:
            best_i, best_d = lo - 1, abs(rx_times[lo - 1] - t)
        if best_d <= tolerance:
            matched.append((tx, rx_rows[best_i], best_d))
    return matched


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--tx', required=True)
    p.add_argument('--rx', required=True)
    p.add_argument('--out', default='combined.csv')
    p.add_argument('--tolerance', type=float, default=0.5)
    p.add_argument('--leap', type=int, default=18)
    args = p.parse_args()

    print("=" * 62)
    print("  Combining transmitter + receiver logs by GPS time")
    print("=" * 62)

    print("\nTRANSMITTER FILE:")
    tx = load_transmitter(args.tx, args.leap)

    print("\nRECEIVER FILE:")
    rx = load_receiver(args.rx)

    if not tx:
        print("\nNo transmitter rows with a valid GPS fix were found.")
        print("Re-record outside with the FINAL sketch until it shows FIX OK.")
        return
    if not rx:
        print("\nNo usable receiver rows.")
        return

    print(f"\nTransmitter span: {gps_seconds_to_readable(tx[0]['gps_seconds'], args.leap)}"
          f"  ->  {gps_seconds_to_readable(tx[-1]['gps_seconds'], args.leap)}")
    print(f"Receiver span:    {gps_seconds_to_readable(rx[0]['gps_seconds'], args.leap)}"
          f"  ->  {gps_seconds_to_readable(rx[-1]['gps_seconds'], args.leap)}")

    ov_start = max(tx[0]['gps_seconds'], rx[0]['gps_seconds'])
    ov_end   = min(tx[-1]['gps_seconds'], rx[-1]['gps_seconds'])
    if ov_end < ov_start:
        print("\n*** THE TWO LOGS DO NOT OVERLAP IN TIME ***")
        print(f"    They are {abs(ov_start - ov_end):.0f} seconds apart.")
        print("    Both devices must be recording at the same moment.")
        return
    print(f"Overlap: {ov_end - ov_start:.1f} seconds")

    pairs = match(tx, rx, args.tolerance)
    print(f"\nMatched pairs: {len(pairs)}")
    if not pairs:
        print("No pairs within tolerance. Try --tolerance 2.0")
        return

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['gps_seconds', 'utc_time', 'time_offset_s',
                    'tx_blink', 'tx_lat', 'tx_lon',
                    'tx_tilt_x', 'tx_tilt_y', 'tx_tilt_z',
                    'rx_lat', 'rx_lon', 'rx_alt',
                    'rx_x_enu', 'rx_y_enu', 'rx_z_enu',
                    'rx_roll_deg', 'rx_pitch_deg', 'rx_yaw_deg',
                    'rx_ins_status'])
        for t, r, d in pairs:
            w.writerow([
                f"{t['gps_seconds']:.3f}",
                gps_seconds_to_readable(t['gps_seconds'], args.leap),
                f"{d:.3f}",
                t.get('blink_number', ''), t.get('lat', ''), t.get('lon', ''),
                t.get('tilt_x', ''), t.get('tilt_y', ''), t.get('tilt_z', ''),
                r.get('lat', ''), r.get('lon', ''), r.get('alt', ''),
                r.get('x_enu', ''), r.get('y_enu', ''), r.get('z_enu', ''),
                r.get('roll_deg', ''), r.get('pitch_deg', ''), r.get('yaw_deg', ''),
                r.get('ins_status', ''),
            ])

    print(f"Worst mismatch in a pair: {max(d for _, _, d in pairs):.3f} s")
    print(f"\nSaved: {args.out}")


if __name__ == '__main__':
    main()
