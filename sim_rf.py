#!/usr/bin/env python3

"""
Simulate a Ping sonar rangefinder.
"""

import argparse
import csv
import time
from typing import NamedTuple

from pymavlink import mavutil
from pymavlink.dialects.v10.ardupilotmega import MAVLINK_MSG_ID_LOCAL_POSITION_NED

# Connect to BlueOS (not tested)
# CONN_STR = 'udpout:localhost:9000'

# Connect to mavproxy in my localhost test
# 14550 is used by QGC, so we'll use 14551
CONN_STR = 'udpin:localhost:14551'

MIN_MEASUREMENT_M = 0.2
MIN_MEASUREMENT_CM = int(MIN_MEASUREMENT_M * 100)

MAX_MEASUREMENT_M = 50.0
MAX_MEASUREMENT_CM = int(MAX_MEASUREMENT_M * 100)

SENSOR_TYPE = mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN
SENSOR_ID = 1
ORIENTATION = mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270  # Downward-facing
COVARIANCE = 0  # Ping + BlueOS behavior

# A Ping outlier that I've seen: if the sensor reading is < 0.35m, then return ~8m
OUTLIER_LOW_CUTOFF = 0.35
OUTLIER_LOW_READING = 8.888

# Vertical distance from base (sub body origin) to Ping sonar
# TODO depends on what ArduSub uses as body origin -- is it baro_z? Or something else?
BASE_PING_Z = -0.095


class TimeSync(NamedTuple):
    """Match ArduSub time (time since boot in ms) to time.time()"""
    time_boot_ms: int
    wall_time_s: float


class SubZHistory:
    """Keep track of recent LOCAL_POSITION_NED.z readings so that we can simulate a delay"""

    def __init__(self):
        # History is a list of tuples (t, z)
        self.history: list[tuple[float, float]] = []

    def add(self, t: float, sub_z: float):
        # TODO trim older readings to save time and space in long simulations
        self.history.append((t, sub_z))

    def get(self, t: float) -> float or None:
        """Return the z reading at time t. Return None if there is no good z reading."""

        if len(self.history) == 0:
            return None

        # We can't get a reading in the past
        if t < self.history[0][0]:
            return None

        if len(self.history) == 1:
            return self.history[0][1]

        for i in range(1, len(self.history)):
            if t < self.history[i][0]:
                # We're between 2 readings, interpolate
                t1, d1 = self.history[i - 1]
                t2, d2 = self.history[i]
                return d1 + (d2 - d1) * (t - t1) / (t2 - t1)

        # We fell off the end, use the last reading
        return self.history[len(self.history) - 1][1]


class RangefinderSimulator:
    def __init__(self, terrain, delay, outliers):
        self.terrain = terrain
        self.delay = delay
        self.outliers = outliers
        self.conn = mavutil.mavlink_connection(CONN_STR)

        self.sub_z_history = SubZHistory()

        # Sync ArduSub time and wall time on the first LOCAL_POSITION_NED message
        self.time_sync: TimeSync or None = None

    # Keep track of ArduSub time_boot_ms
    def time_boot_ms(self) -> int:
        if self.time_sync is None:
            # Still waiting on a LOCAL_POSITION_NED message
            return 0
        else:
            return self.time_sync.time_boot_ms + int((time.time() - self.time_sync.wall_time_s) * 1000)

    def run(self):
        # Open the output file once
        with open('stamped_terrain.csv', mode='w', newline='') as outfile:
            datawriter = csv.writer(outfile, delimiter=',', quotechar='|')
            datawriter.writerow(['TimeUS', 'terrain_z', 'sub_z', 'rf'])
    
            # Continue until the user hits ctrl-C
            while True:

                # Re-open the input file so the sequence repeats forever
                with open(self.terrain, newline='') as infile:
                    datareader = csv.reader(infile, delimiter=',', quotechar='|')
    
                    # The first input row is the interval
                    row = next(datareader)
                    interval = float(row[0])
    
                    for row in datareader:
                        # Get all LOCAL_POSITION_NED messages that are sitting in the queue
                        # Note: run QGroundControl to get the LOCAL_POSITION_NED messages flowing
                        while msg := self.conn.recv_match():
                            if msg.get_msgId() == MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                                timestamp = getattr(msg, '_timestamp', 0.0)

                                # Synchronize clocks
                                if self.time_sync is None:
                                    self.time_sync = TimeSync(msg.time_boot_ms, timestamp)

                                # Add the reading to the depth history
                                self.sub_z_history.add(timestamp, -msg.z)

                        # terrain_z is above/below seafloor depth
                        terrain_z = float(row[0])

                        # Get the sub.z reading at time t, where t = now - delay
                        sub_z = self.sub_z_history.get(time.time() - self.delay)

                        if sub_z is None:
                            # Send an out-of-range measurement
                            rf = 0.0
                        else:
                            # sub_z = -7, terrain_z = -18, reading = 11
                            # Adjust for distance from sub body origin to location of Ping sonar
                            rf = sub_z - terrain_z + BASE_PING_Z

                            if rf < OUTLIER_LOW_CUTOFF and self.outliers:
                                rf = OUTLIER_LOW_READING
                            elif rf < MIN_MEASUREMENT_M:
                                rf = MIN_MEASUREMENT_M
                            elif rf > MAX_MEASUREMENT_M:
                                rf = MAX_MEASUREMENT_M

                        print(f'terrain_z {terrain_z}, sub_z {sub_z}, rf {rf}')
    
                        self.conn.mav.distance_sensor_send(
                            0,              # time_boot_ms is ignored by AP_RangeFinder_MAVLink
                            MIN_MEASUREMENT_CM,
                            MAX_MEASUREMENT_CM,
                            int(rf * 100),  # Convert m -> cm
                            SENSOR_TYPE,
                            SENSOR_ID,
                            ORIENTATION,
                            COVARIANCE)

                        # For logging purposes, record bad sub_z measurements as 0.0
                        log_sub_z = 0.0 if sub_z is None else sub_z

                        # Generate TimeUS (time-since-boot in microseconds) to match the CTUN msg
                        datawriter.writerow([self.time_boot_ms() * 1000, terrain_z, log_sub_z, rf])
                        outfile.flush()

                        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delay', type=float, default=0.8, help='Ping sensor delay in seconds')
    parser.add_argument('--terrain', type=str, default='terrain/zeros.csv', help='terrain file')
    parser.add_argument('--outliers', action='store_true', help='add outliers')
    args = parser.parse_args()
    print(f'terrain: {args.terrain}, Ping sensor delay: {args.delay}s, outliers: {args.outliers}')
    
    simulator = RangefinderSimulator(args.terrain, args.delay, args.outliers)
    simulator.run()
    

if __name__ == '__main__':
    main()