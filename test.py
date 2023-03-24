#!/bin/python3

import argparse
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


def drone_run(drone_interface: DroneInterface):
    """ Test script for the drone"""

    ##### ARM OFFBOARD #####
    drone_interface.arm()
    drone_interface.offboard()

    ##### TAKE OFF #####
    print("Take Off")
    if not drone_interface.takeoff(height=4.0, speed=0.5):
        print("Take Off failed")
        return
    print("Take Off done")
    sleep(10.0)

    print("Go to Point")
    if not drone_interface.go_to.go_to_point_path_facing(
            [10.0, 0.0, 2.0], speed=1.0):
        print("Go to Point failed")
        return
    print("Goto done")
    sleep(2.0)

    print("Go to Return")
    if not drone_interface.go_to.go_to_point_path_facing(
            [0.0, 0.0, 2.0], speed=1.0):
        print("Go to Return failed")
        return
    print("Go to return done")
    sleep(2.0)

    ##### LAND #####
    print("Landing")
    if not drone_interface.land(speed=0.5):
        print("Landing failed")
        return
    print("Land done")


if __name__ == '__main__':
    # Get the namespace of the drone by parsing the arguments
    parser = argparse.ArgumentParser(description='Test script for the drone')
    parser.add_argument('--ns', type=str, help='Namespace of the drone')
    args = parser.parse_args()
    uav_name = args.ns

    rclpy.init()

    uav = DroneInterface(uav_name, verbose=True, use_sim_time=False)
    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
