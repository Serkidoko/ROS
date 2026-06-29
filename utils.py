import time
import math
import cv2
import numpy as np
import copy

# === FIXED CONFIGURATION ===
PULSES_PER_REV = 1850          # measured encoder pulses per revolution
WHEEL_DIAMETER = 0.065         # meters (e.g., 65 mm)
CALIBRATION_FACTOR = 1.0       # overall distance calibration factor
READ_INTERVAL = 0.05           # seconds between encoder reads
YAW_KP = 10.0                  # proportional gain for yaw correction (tune as needed)
YAW_TARGET = 0.0               # target yaw in degrees
STEER_KP = 5.0                 # P gain for steering correction
STEER_KI = 0.5                 # I gain for steering correction
MOTION_KP = 1.2                # increased P for better velocity tracking
MOTION_KI = 0.1                # slightly increased I
MOTION_KD = 0.1                # small D term to damp oscillations
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi * CALIBRATION_FACTOR
MOTOR_CALIBRATION = [0.990, 0.995, 1.017, 0.992]    # when pin position is middle inside the body


def ticks_to_meters(ticks):
    """Convert encoder ticks to linear distance in meters."""
    revolutions = ticks / PULSES_PER_REV
    return revolutions * WHEEL_CIRCUMFERENCE

def normalize_angle(angle):
    """Normalize angle to (-180, 180] degrees"""
    return ((angle + 180) % 360) - 180

def calib_range(dist_calib, bb_pxl, adj):
    position = 320 - (bb_pxl[0] + bb_pxl[2]/2)
    dist_adjust = position * dist_calib
    dist_adjust = dist_adjust - adj if dist_adjust > 0 else dist_adjust - adj + 0.03
    speed_adjust = 20 if dist_adjust > 0 else -20
    return speed_adjust, dist_adjust

def shortest_angle_diff(current, target):
    """Returns the smallest difference from current to target (in degrees), can be negative"""
    diff = normalize_angle(target - current)
    return diff

def init_arm(bot, delay=1):
    # hide arm from depth cam FOV
    bot.set_uart_servo_angle_array([167, 180, 0, 0, 90, 42], 
                                   run_time=8000) 
    time.sleep(delay)
    
def gripe_box_right(bot, pose_angles:tuple, pose2_angles: tuple, gripe_angles: tuple, hold_angles: tuple):
    """
        MUST SPECIFY bot as an instance of Rosmaster FIRST
        Move the arm to the right side to grip a box
    """
    print("[INFO] Moving arm to grip right side box...")
    print("Setting arm servo angles to:", gripe_angles)
    bot.set_uart_servo_torque(True)
    
    # gripe the box
    bot.set_uart_servo_angle(1, 180, run_time=8000)
    time.sleep(2)
    bot.set_uart_servo_angle_array(pose_angles, run_time=8000)
    time.sleep(2)
    bot.set_uart_servo_angle_array(gripe_angles, run_time=8000)
    time.sleep(2)
    bot.set_uart_servo_angle(6, 132)
    time.sleep(1)

    # hold the box
    print("Setting arm servo angles to:", hold_angles)
    bot.set_uart_servo_angle(2, 90, run_time=8000)
    time.sleep(2)
    bot.set_uart_servo_angle_array(pose2_angles, run_time=8000)
    time.sleep(2)
    bot.set_uart_servo_angle_array(hold_angles, run_time=8000)
    time.sleep(1)

def check_rotate_direction(init_yaw, target_yaw):
    direction_left = None
    print(f"[INFO] Rotating from yaw {init_yaw:.2f} to target yaw {target_yaw:.2f} degrees")
        
    if target_yaw - init_yaw > 0 and target_yaw - init_yaw < 180: direction_left = True
    elif target_yaw - init_yaw < 0 and target_yaw - init_yaw > -180: direction_left = False
    elif target_yaw - init_yaw > 180: direction_left = False
    elif target_yaw - init_yaw < -180: direction_left = True
    
    return direction_left


def rotate_robot(bot, angle, spd=0.2, delay=1):
    _, _, init_yaw = bot.get_imu_attitude_data()
    yaw = init_yaw + angle
    yaw = normalize_angle(yaw)
    direction_left = check_rotate_direction(init_yaw,  yaw)
    angular_speed = spd if direction_left else -spd 
    bot.set_car_motion(0, 0, angular_speed)
    
    while True:
        _, _, current_yaw = bot.get_imu_attitude_data()
        current_yaw = normalize_angle(current_yaw)
        diff = shortest_angle_diff(current_yaw, yaw)
        # print(diff)

        if abs(diff) <= 0.5: break

        time.sleep(READ_INTERVAL)

    bot.set_car_motion(0, 0, 0)
    time.sleep(delay)
    print(f"[INFO] Rotation completed.\nCurrent yaw: {current_yaw:.2f} degrees | Target yaw: {yaw:.2f} degrees")
    
    
def run_straight_robot(bot, target_yaw, base_speed=37, target_distance=0.1, delay=1):
    # configure built-in motor velocity PID
    bot.set_pid_param(MOTION_KP, MOTION_KI, MOTION_KD, forever=False)
    kp, ki, kd = bot.get_motion_pid()
    print(f"[INFO] Motor PID set to Kp={kp}, Ki={ki}, Kd={kd}")

    # read initial encoder ticks for all 4 motors (M1=LF, M2=LR, M3=RF, M4=RR)
    start_ticks = list(bot.get_motor_encoder())
    print(f"[INFO] Initial ticks: {start_ticks}")

    error_int = [0.0] * 4
    traveled = 0.0
    debug_bool = 1.0
    print(f"[INFO] Moving {target_distance:.2f} m at base speed {base_speed}")

    try:
        while abs(traveled) < abs(target_distance):
            enc = list(bot.get_motor_encoder())
            deltas = [enc[i] - start_ticks[i] for i in range(4)]
            distances = [ticks_to_meters(d) for d in deltas]
            traveled = sum(distances) / 4.0

            # steering errors and integral for each wheel
            errors = [d - traveled for d in distances]
            for i in range(4):
                error_int[i] += errors[i] * READ_INTERVAL

            # PI steering corrections
            corrections = [STEER_KP * errors[i] + STEER_KI * error_int[i] for i in range(4)]

            # get IMU data
            _, _, yaw = bot.get_imu_attitude_data()
            yaw_error = yaw - target_yaw
            yaw_correction = YAW_KP * yaw_error

            # Apply yaw correction: subtract from left wheels, add to right wheels
            speeds = []
            for i in range(4):
                if i in [0, 1]:  # Left wheels (LF, LR)
                    spd_cmd = (base_speed - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
                    # spd_cmd = (base_speed - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
                else:            # Right wheels (RF, RR)
                    spd_cmd = (base_speed - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
                    # spd_cmd = (base_speed - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
                speeds.append(spd_cmd)

            # send speed commands; set_motor uses velocity PID internally
            bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])

            # debug info
            if traveled >= debug_bool:
                debug_bool += 0.5
                print(f"[DEBUG] Dists: {[f'{d:.3f}' for d in distances]}, Avg: {traveled:.3f}")
                print(f"        Speeds: {[f'{s:.1f}' for s in speeds]}, Errors: {[f'{e:.3f}' for e in errors]}")
                print(f"        IMU Yaw: {yaw:.3f}, Error: {yaw_error:.3f}, Correction: {yaw_correction:.3f}")
            
            time.sleep(READ_INTERVAL)

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    finally:
        bot.set_motor(0, 0, 0, 0)
        time.sleep(delay)
        print(f"[INFO] Stopped at {traveled:.3f} m.")
        

def run_Cam_thread(id=2, height=480, width=640, height_ratio=0.12, process_every=10, color_ranges=None, detected_flag=None, bounding_boxes=None, lock=None):
    cap = cv2.VideoCapture(id)
    if not cap.isOpened():
        print(f'Can not open Camera {id}')
        return

    frame_count = 0
    start_time = time.time()
    box_detection_counts = {}  # (color, x, y, w, h) -> count
    color_confirmed_boxes = {}  # color -> (x, y, w, h)

    while time.time() - start_time < 10:
        ret, frame = cap.read()
        if not ret:
            print(f'Can not receive frame from camera {id}')
            break

        box_top = height - int(height_ratio * height)
        box_bottom = height
        box_left = 0
        box_right = width

        frame_origin = copy.deepcopy(frame)
        cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (255, 0, 0), 2)
        cv2.putText(frame, f'Width: {width}, Height: {height}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if frame_count % process_every == 0:
            frame_count = 1
            roi = frame[box_top:box_bottom, box_left:box_right]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            detected_any_box = False

            for color, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    y_abs = y + box_top
                    if w * h > 800 and w < 50 and h < 50 and w > 20 and h > 20:
                        box_key = (color, x, y_abs, w, h)
                        box_detection_counts[box_key] = box_detection_counts.get(box_key, 0) + 1
                        detected_any_box = True

                        # Confirm box if detected more than 3 times
                        if box_detection_counts[box_key] > 3:
                            color_confirmed_boxes[color] = (x, y_abs, w, h)
                            cv2.rectangle(frame, (x, y_abs), (x + w, y_abs + h), (0, 0, 255), 2)
                            cv2.putText(frame, color, (x, y_abs - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Set detected_flag[0] as soon as any box is detected
            if detected_any_box and not detected_flag[0]:
                with lock:
                    # time.sleep(0.05)
                    detected_flag[0] = True

            # Update bounding_boxes with the most recent confirmed boxes (one per color)
            if color_confirmed_boxes:
                with lock:
                    # detected_flag[0] = True
                    bounding_boxes.clear()
                    for color, box in color_confirmed_boxes.items():
                        bounding_boxes.append((color, box))
                
                    if len(color_confirmed_boxes) == 3: break

        cv2.imshow('Depth Camera', frame)
        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.imwrite('./view_origin.jpg', frame_origin)
    cv2.imwrite('./view_detect.jpg', frame)
    cap.release()
    cv2.destroyAllWindows()


def justify_straight_thread(bot, init_yaw, base_speed, detected_flag, traveled_result, lock):
    bot.set_pid_param(MOTION_KP, MOTION_KI, MOTION_KD, forever=False)
    start_ticks = list(bot.get_motor_encoder())
    error_int = [0.0] * 4
    traveled = 0.0

    while True:
        speeds = []
        with lock:
            # print(f'Speeds 0: {speeds}', end=' | ')
            if detected_flag[0]:
                # print(f'Speeds end: {speeds}')
                time.sleep(0.05)
                break

        enc = list(bot.get_motor_encoder())
        deltas = [enc[i] - start_ticks[i] for i in range(4)]
        distances = [ticks_to_meters(d) for d in deltas]
        traveled = sum(distances) / 4.0

        errors = [d - traveled for d in distances]
        for i in range(4):
            error_int[i] += errors[i] * READ_INTERVAL

        corrections = [STEER_KP * errors[i] + STEER_KI * error_int[i] for i in range(4)]
        _, _, yaw = bot.get_imu_attitude_data()
        yaw_error = yaw - init_yaw
        yaw_correction = YAW_KP * yaw_error

        for i in range(4):
            if i in [0, 1]:
                spd_cmd = (base_speed - corrections[i] + yaw_correction) * MOTOR_CALIBRATION[i]
            else:
                spd_cmd = (base_speed - corrections[i] - yaw_correction) * MOTOR_CALIBRATION[i]
            speeds.append(max(0, min(42, spd_cmd)))

        # print(f'Speeds 1: {speeds}')
        bot.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])
        
        if traveled >= 5: 
            print(f"[DEBUG] Justified straight too long => stop!!!")
            break

        time.sleep(READ_INTERVAL)
    
    bot.set_motor(0, 0, 0, 0)
    time.sleep(2.4)
    traveled_result.append(traveled)