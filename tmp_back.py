# Dependecies and Setup

import argparse, threading
from Rosmaster_Lib import Rosmaster
from utils import *


parser = argparse.ArgumentParser(description="Robot color detection")
parser.add_argument('--color1', type=str, choices=['red', 'green', 'yellow'], default='red', help='Target box A color')
parser.add_argument('--color2', type=str, choices=['red', 'green', 'yellow'], default='red', help='Target box C color')
parser.add_argument('--adjust1', type=float, default=0.13, help='Dist adjust 1')
parser.add_argument('--adjust2', type=float, default=0.13, help='Dist adjust 2')
args = parser.parse_args()
color1 = args.color1
color2 = args.color2
adj1 = args.adjust1
adj2 = args.adjust2

dist_calib = 0.22 / 220
pose_angles = (180, 120, 90, 75, 90, 42)
grip_angles = (180, 85, 90, 75, 90, 42)
pose2_angles = (180, 120, 90, 75, 90, 132)
hold_angles = (90, 180, 0, 90, 90, 132)

color_ranges = {
    'red':   [(170, 70, 50), (180, 255, 255)],
    'green': [(30, 60, 50), (85, 255, 255)],
    # 'green': [(40, 80, 50), (85, 255, 255)],
    'yellow':[(10, 100, 100), (35, 255, 255)],
}


# Init Robot
bot = Rosmaster()
bot.clear_auto_report_data()
bot.create_receive_threading()

print(f"Voltage remain: {bot.get_battery_voltage()}")
_, _, init_yaw = bot.get_imu_attitude_data()
# init_yaw = map_imu_angle(init_yaw)
print('Initial Yaw:', init_yaw)
init_arm(bot, delay=1)

# Step 1: Move to Box A
rotate_robot(bot, -10, spd=0.2, delay=1)      # Rotate RIGHT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=2.06)
rotate_robot(bot, 100, spd=0.2, delay=1)      # Rotate LEFT

# Step 2: Calib + Gripe Box A
detected_flag = [False]
bb = []
traveled_result = []
_, _, yaw = bot.get_imu_attitude_data()
lock = threading.Lock()


t1 = threading.Thread(target=justify_straight_thread, args=(bot, yaw, 20, detected_flag, traveled_result, lock))
t2 = threading.Thread(target=run_Cam_thread, args=(2, 480, 640, 0.12, 1, color_ranges, detected_flag, bb, lock))
t1.start()
t2.start()
t1.join()
t2.join()
print(f'[INFO] {bb}')

if bb: 
    print(f'[INFO] Detected bounding boxes: {bb} after running {traveled_result[0]:.2f}')

    _, _, yaw = bot.get_imu_attitude_data()
    run_straight_robot(bot, target_yaw=yaw, base_speed=20, target_distance=0.02)
    rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT

    for box in bb:
        if box[0] == color1:
            speed_adjust, dist_adjust = calib_range(dist_calib, box[1], adj=adj1)
            print(f'[INFO] Adjusted distance requirement: {dist_adjust:.2f} m')
    
    _, _, yaw = bot.get_imu_attitude_data()
    run_straight_robot(bot, target_yaw=yaw, base_speed=speed_adjust, target_distance=dist_adjust)
    gripe_box_right(bot, pose_angles, pose2_angles, grip_angles, hold_angles)


# Step 3: Move to Box B
_, _, yaw = bot.get_imu_attitude_data()
rotate_robot(bot, init_yaw + 180 - yaw, spd=0.2, delay=1)      # Rotate LEFT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=0.5 - dist_adjust)
rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT

_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=2.7)

bot.set_uart_servo_torque(True)
hold2_angles = (90, 145, 30, 90, 90, 132)
bot.set_uart_servo_angle_array(hold2_angles, run_time=8000)
time.sleep(2)

bot.set_uart_servo_angle(6, 42)
time.sleep(1)


# Step 4: Move to Box C
rotate_robot(bot, 60, spd=0.2, delay=1)      # Rotate LEFT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=-20, target_distance=0.5)
rotate_robot(bot, -60, spd=0.2, delay=1)      # Rotate RIGHT
init_arm(bot, delay=1)


# Step 5: Calib + Gripe Box C
detected_flag = [False]
bb = []
traveled_result = []
_, _, yaw = bot.get_imu_attitude_data()
lock = threading.Lock()

t1 = threading.Thread(target=justify_straight_thread, args=(bot, yaw, 20, detected_flag, traveled_result, lock))
t2 = threading.Thread(target=run_Cam_thread, args=(2, 480, 640, 0.12, 1, color_ranges, detected_flag, bb, lock))
t1.start()
t2.start()
t1.join()
t2.join()
print(f'[INFO] {bb}')

if bb: 
    print(f'[INFO] Detected bounding boxes: {bb} after running {traveled_result[0]:.2f}')

    _, _, yaw = bot.get_imu_attitude_data()
    run_straight_robot(bot, target_yaw=yaw, base_speed=20, target_distance=0.01)
    rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT

    for box in bb:
        if box[0] == color2:
            speed_adjust, dist_adjust = calib_range(dist_calib, box[1], adj=adj2)
            print(f'[INFO] Adjusted distance requirement: {dist_adjust:.2f} m')
    
    _, _, yaw = bot.get_imu_attitude_data()
    run_straight_robot(bot, target_yaw=yaw, base_speed=speed_adjust, target_distance=dist_adjust)
    gripe_box_right(bot, pose_angles, pose2_angles, grip_angles, hold_angles)


# Step 6: Move to Box D
# _, _, yaw = bot.get_imu_attitude_data()
# rotate_robot(bot, init_yaw + 90 - yaw, spd=0.2, delay=1)      # Rotate LEFT
rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=2.5)
rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT

_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=0.9 + dist_adjust)

bot.set_uart_servo_torque(True)
hold2_angles = (90, 145, 30, 90, 90, 132)
bot.set_uart_servo_angle_array(hold2_angles, run_time=8000)
time.sleep(2)

bot.set_uart_servo_angle(6, 42, run_time=8000)


# Step 7: Move to HOME
rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=0.5)
rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=0.3)

# Terminate the robot's receive thread and clean up