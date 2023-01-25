#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface_gps import DroneInterfaceGPS as DroneInterface


def drone_run(drone_interface: DroneInterface):

    origin = [40.158194, -3.380795, 0]

    sleep_time = 2.0

    drone_interface.gps.set_origin(origin)
    print("ORIGIN at", drone_interface.gps.origin)
    print("Start mission")

    # ##### ARM OFFBOARD #####
    drone_interface.arm()
    drone_interface.offboard()

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(height=2.0, speed=0.5)
    print("Take Off done")
    sleep(sleep_time)

    print("Go to Point")
    drone_interface.goto.go_to_gps_point(
        [40.158183, -3.380893, 2.0], speed=1.0)
    print("Goto done")
    sleep(sleep_time)

    print("Go to Return")
    drone_interface.goto.go_to_gps_point(
        [40.158194, -3.380795, 2.0], speed=1.0)
    print("Go to return done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    # drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()
    uav_name = "drone0"
    uav = DroneInterface(uav_name, verbose=False, use_sim_time=False)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
