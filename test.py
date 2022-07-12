#!/bin/python3

import rclpy
import sys
import numpy as np
from time import sleep
import threading
from python_interface.drone_interface import DroneInterface
from as2_msgs.msg import TrajectoryWaypoints

n_uavs = 1
offset = 0


def drone_run(drone_interface, n_uav):

    speed = 2.0
    takeoff_height = 2.0
    height = 3.0

    # goal0 = [243.0, 0.00, height]
    # goal1 = [243.0, 127.00, height]

    goto_list = [
        [5.0,  0.0, height],
        [0.0, -5.0, height],
        [-5.0,  0.0, height],
        [5.0,  5.0, height],
    ]

    # goto_list =  [
    # [  0.000000,  0.000000],
    # [ 58.853893,  1.376182],
    # [ 24.009415,-92.325882],
    # [ 81.315829,-90.851292],
    # [ 76.901956,  5.406108],
    # [180.016960,  8.224816],
    # [ 90.339703,-59.202171],
    # [209.299196,-35.971765]]

    print(f"Start mission {n_uav}")

    drone_interface.offboard()
    print("OFFBOARD")
    sleep(5)
    drone_interface.arm()
    print("ARMED")
    sleep(5)
    print(f"Take Off {n_uav}")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print(f"Take Off {n_uav} done")
    sleep(10)

    # sleep(1.0)
    # print(f"Go to {n_uav}: [{goal0},0.0,{height}]")
    # drone_interface.go_to_point(goal0, speed=speed, ignore_yaw=False)
    # print(f"Go to {n_uav} done")

    # sleep(1.0)
    # print(f"Go to {n_uav}: [{goal1},0.0,{height}]")
    # drone_interface.go_to_point(goal1, speed=speed, ignore_yaw=False)
    # print(f"Go to {n_uav} done")

    # goto_list = [[0, 0]]

    for wp in goto_list:
        print(f"Go to {n_uav}: [{wp[0]},{wp[1]},{height}]")
        drone_interface.go_to_point(
            [wp[0], wp[1], height], speed=speed, ignore_yaw=False)
        print(f"Go to {n_uav} done")
        sleep(4.0)

    # sleep(1.0)
    # print(f"Follow path {n_uav}")
    # path = [goal0, goal1, [0, 0, height]]
    # drone_interface.follow_path(path, speed=speed)
    # print(f"Follow path {n_uav} done")

    sleep(1.0)
    print(f"Land {n_uav}")
    drone_interface.land(speed=0.5)
    print(f"Land {n_uav} done")

    print("Clean exit")


if __name__ == '__main__':
    rclpy.init()
    n_uavs = DroneInterface("drone0", verbose=True)

    drone_run(n_uavs, 0)

    n_uavs.shutdown()
    rclpy.shutdown()
    exit(0)


# 28.14376, -16.50235

# 0: "lat":28.143989765431503,"lng":-16.503195203840733
# 1: "lat":28.144002182090170,"lng":-16.502596065402034
# 2: "lat":28.143156663474590,"lng":-16.502950787544254
# 3: "lat":28.143169967140846,"lng":-16.502367407083515
# 4: "lat":28.144038545153656,"lng":-16.502412334084514
# 5: "lat":28.144063969727295,"lng":-16.501362584531310
# 6: "lat":28.143455552111522,"lng":-16.502275541424755
# 7: "lat":28.143665157987456,"lng":-16.501064524054530

# 0: [0.0, 0.0]
# 1: [58.858650072943420, 0.6475555878132582]
# 2: [22.861856688745320, -92.60375464893878]
# 3: [80.174562452710230, -91.83864222606644]
# 4: [76.952817701967430, 4.4532878105528650]
# 5: [180.08389804034960, 5.9952842774800960]
# 6: [89.588214213959870, -60.30780006898567]
# 7: [208.81032733654138, -38.55439674202353]

# 0: [  0.000,  0.000]
# 1: [ 58.853,  1.376]
# 2: [ 24.009,-92.325]
# 3: [ 81.315,-90.851]
# 4: [ 76.901,  5.406]
# 5: [180.016,  8.224]
# 6: [ 90.339,-59.202]
# 7: [209.299,-35.971]
