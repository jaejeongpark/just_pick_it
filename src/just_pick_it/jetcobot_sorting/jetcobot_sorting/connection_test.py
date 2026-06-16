#!/usr/bin/env python3

import time
from pymycobot.mycobot280 import MyCobot280


def main():
    port = "/dev/ttyJETCOBOT"
    baudrate = 1000000
    speed = 20

    mc = MyCobot280(port, baudrate)

    print(f"Connected to {port} at {baudrate}")

    # 로봇 전원 ON
    mc.power_on()
    time.sleep(1)

    # 현재 각도 확인
    angles = mc.get_angles()
    print("Current angles:", angles)

    if not angles:
        print("Failed to read angles. Check port, baudrate, power, or firmware.")
        return

    # 0 자세로 이동
    target_angles = [0, 0, 0, 0, 0, 0]
    print("Sending target angles:", target_angles)

    mc.send_angles(target_angles, speed)

    time.sleep(5)

    # 이동 후 각도 확인
    angles_after = mc.get_angles()
    print("Angles after motion:", angles_after)


if __name__ == "__main__":
    main()