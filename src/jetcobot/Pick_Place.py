import time
from pymycobot.mycobot280 import MyCobot280

mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

SPEED = 30
WAIT = 4

home_joints  = [0, 0, 0, 0, 0, 0]
pick_coords  = [-230.4, -100.1, 215.5, -170.71, -2.27, 140.79]
lift_coords  = [-161.2, -50.7, 309.2, -153.92, -3.34, 140.86]
place_coords = [4.8, -272.1, 170.3, -171.88, -1.76, -130.51]


def move_to_coords(coords, label):
    print(f"\n▶ {label}")
    start = time.time()
    mc.send_coords(coords, SPEED, mode=1)
    time.sleep(WAIT)
    elapsed = time.time() - start
    print(f"  현재 좌표: {mc.get_coords()}")
    print(f"  목표 좌표: {coords}")
    print(f"  소요시간: {elapsed:.1f}초")
    print("  ✅ 이동 완료")
    return True


def move_to_home(label="Home으로 복귀"):
    print(f"\n▶ {label}")
    start = time.time()
    mc.send_angles(home_joints, SPEED)
    time.sleep(WAIT)
    elapsed = time.time() - start
    print(f"  소요시간: {elapsed:.1f}초")
    print("  ✅ 이동 완료")
    return True


def gripper_open():
    print("\n▶ 그리퍼 열기")
    start = time.time()
    mc.set_gripper_value(100, SPEED)
    time.sleep(WAIT)
    elapsed = time.time() - start
    print(f"  소요시간: {elapsed:.1f}초")


def gripper_close():
    print("\n▶ 그리퍼 닫기")
    start = time.time()
    mc.set_gripper_value(0, SPEED)
    time.sleep(WAIT)
    elapsed = time.time() - start
    print(f"  소요시간: {elapsed:.1f}초")


print("=" * 40)
print("Pick & Place 작업 시작")
print("=" * 40)

move_to_coords(pick_coords,  "1. Pick 위치로 이동")
gripper_close()
move_to_coords(lift_coords,  "2. 위로 올리기")
move_to_coords(place_coords, "3. Place 위치로 이동")
gripper_open()
move_to_home("4. Home으로 복귀")

print("\n" + "=" * 40)
print("✅ Pick & Place 작업 완료!")
print("=" * 40)