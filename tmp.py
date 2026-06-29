# Dependecies and Setup

import argparse, threading
from Rosmaster_Lib import Rosmaster
from utils import *


parser = argparse.ArgumentParser(description="Robot color detection")
parser.add_argument('--color', type=str, choices=['red', 'green', 'yellow'], default='red', help='Target color')
args = parser.parse_args()
color = args.color


dist_calib = 0.22 / 220
pose_angles = (180, 120, 90, 75, 90, 42)
grip_angles = (180, 85, 90, 75, 90, 42)
pose2_angles = (180, 120, 90, 75, 90, 132)
hold_angles = (90, 180, 0, 90, 90, 132)
# color_ranges = {
#     'red': [(160, 45, 15), (180, 255, 255)],
#     'green': [(35, 50, 20), (85, 255, 255)],
#     'yellow':[(10, 100, 100), (35, 255, 255)],
# }
color_ranges = {
    #'red':   [(170, 70, 50), (180, 255, 255)],
    # 'green': [(35, 40, 40),  (85, 255, 255)],
    #'green': [(35, 80, 50), (85, 255, 255)],
    'yellow':[(10, 100, 100), (35, 255, 255)],
    'red':   [(0, 45, 35),   (12, 255, 255)],
    'green': [(45, 45, 30),  (90, 255, 255)],
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
time.sleep(1)


# Step 1: Move to A
rotate_robot(bot, -10, spd=0.2, delay=1)      # Rotate RIGHT
_, _, yaw = bot.get_imu_attitude_data()
run_straight_robot(bot, target_yaw=yaw, base_speed=37, target_distance=1.96)
rotate_robot(bot, 100, spd=0.2, delay=1)      # Rotate LEFT

# # Step 2: Calib + Gripe Box A
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
    run_straight_robot(bot, target_yaw=yaw, base_speed=20, target_distance=0.036)
    rotate_robot(bot, 90, spd=0.2, delay=1)      # Rotate LEFT

    for box in bb:
        if box[0] == color:
            speed_adjust, dist_adjust = calib_range(dist_calib, box[1], adj=0.2)
            print(f'[INFO] Adjusted distance requirement: {dist_adjust:.2f} m')
    
    _, _, yaw = bot.get_imu_attitude_data()
    run_straight_robot(bot, target_yaw=yaw, base_speed=speed_adjust, target_distance=dist_adjust)
    gripe_box_right(bot, pose_angles, pose2_angles, grip_angles, hold_angles)