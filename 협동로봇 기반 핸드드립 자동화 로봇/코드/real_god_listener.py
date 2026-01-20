#!/usr/bin/env python3
import rclpy
import DR_init
import threading
import time
import subprocess
import os
import signal
import firebase_admin
from firebase_admin import credentials, db
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveStop, SetRobotControl, GetRobotState
from DR_common2 import posj, posx 

# 상수 직접 선언
DR_AXIS_Z = 2
DR_BASE = 0

DR_LIB = None 

# ==========================================================
# 1. 환경 설정
# ==========================================================
FIREBASE_CERT_PATH = "/home/wook/cobot1_ws/src/m0609_monitor/config/serviceAccountKey.json"
DATABASE_URL = "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app"

ORDER_PATH = 'barista_control/order_command'
STATUS_PATH = 'barista_status/current_state'
SPEED_PATH  = 'barista_control/setting/speed_ratio'

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

global pending_order
pending_order = None 

global bringup_process
bringup_process = None

global CURRENT_SPEED_RATIO
CURRENT_SPEED_RATIO = 1.0 

global STOP_EVENT
STOP_EVENT = False

global EXECUTION_MODE, GLOBAL_STEP_COUNT, SAVED_STOP_INDEX, SAVED_ORDER_DATA
EXECUTION_MODE = 'NORMAL'
GLOBAL_STEP_COUNT = 0
SAVED_STOP_INDEX = 0
SAVED_ORDER_DATA = None

global LAST_ROBOT_STATE
LAST_ROBOT_STATE = -1

# [핵심] 복구 진행 중임을 알리는 플래그
global IS_RECOVERING
IS_RECOVERING = False

# [NEW] 감시 전용 노드 전역 변수 (충돌 방지용)
global polling_node
polling_node = None

# ==========================================================
# 2. [안전 장치] 스마트 래퍼 함수
# ==========================================================
def check_stop_signal():
    """정지 신호 감지 시 즉시 예외 발생"""
    global STOP_EVENT, GLOBAL_STEP_COUNT
    if STOP_EVENT:
        print(f" 🛑 [System] STOP Detected! Halting immediately at Step {GLOBAL_STEP_COUNT}.")
        raise Exception("EMERGENCY_STOP_TRIGGERED")

def should_execute_step():
    global GLOBAL_STEP_COUNT, SAVED_STOP_INDEX, EXECUTION_MODE
    
    check_stop_signal() 

    GLOBAL_STEP_COUNT += 1
    
    if EXECUTION_MODE == 'RESUME':
        if GLOBAL_STEP_COUNT < SAVED_STOP_INDEX:
            return False 
        elif GLOBAL_STEP_COUNT == SAVED_STOP_INDEX:
            print(f"   👉 [RESUME] Resuming Action at Step {GLOBAL_STEP_COUNT}!")
            EXECUTION_MODE = 'NORMAL'
            return True
            
    return True

def smart_wait(seconds):
    check_stop_signal()
    start_time = time.time()
    while time.time() - start_time < seconds:
        check_stop_signal()
        time.sleep(0.05)

# --- 동작 함수 래핑 ---
def movej(*args, **kwargs):
    if should_execute_step():
        try:
            if DR_LIB: DR_LIB.movej(*args, **kwargs)
        except: pass
        check_stop_signal()

def movel(*args, **kwargs):
    if should_execute_step():
        try:
            if DR_LIB: DR_LIB.movel(*args, **kwargs)
        except: pass
        check_stop_signal()

def mwait():
    if EXECUTION_MODE == 'NORMAL': 
        check_stop_signal()
        if DR_LIB: DR_LIB.mwait()
        check_stop_signal()

def wait(time_sec):
    if should_execute_step():
        smart_wait(time_sec)

def move_spiral(*args, **kwargs):
    if should_execute_step():
        try:
            if DR_LIB: DR_LIB.move_spiral(*args, **kwargs)
        except: pass
        check_stop_signal()

def initialize_robot_tool(tool_name, tcp_name, vel=100, acc=100):
    if should_execute_step():
        check_stop_signal()
        if DR_LIB:
            DR_LIB.set_tool(tool_name)
            DR_LIB.set_tcp(tcp_name)
        smart_wait(0.5)

def gripper_open():
    print('[GRIPPER] OPEN')
    from DSR_ROBOT2 import set_digital_output, wait
    PIN_CLOSE, PIN_OPEN, PIN_WIDE = 1, 2, 3
    ON, OFF = 1, 0
    set_digital_output(PIN_WIDE,  OFF)   # ★ 이 줄이 핵심(94mm 잔류 방지)
    set_digital_output(PIN_CLOSE, OFF)
    set_digital_output(PIN_OPEN, ON)
    
    wait(1.5)

def gripper_open_wide():
    print('[GRIPPER] OPEN (94mm)')
    from DSR_ROBOT2 import set_digital_output, wait
    PIN_CLOSE, PIN_OPEN, PIN_WIDE = 1, 2, 3
    ON, OFF = 1, 0

    set_digital_output(PIN_CLOSE, OFF)  # IN1=0
    set_digital_output(PIN_OPEN, ON)    # IN2=1
    set_digital_output(PIN_WIDE, ON)    # IN3=1  -> 94mm 조건
    wait(1.5)

def gripper_closed():
    print('[GRIPPER] CLOSE')
    from DSR_ROBOT2 import set_digital_output, wait
    PIN_CLOSE, PIN_OPEN, PIN_WIDE = 1, 2, 3
    ON, OFF = 1, 0
    set_digital_output(PIN_CLOSE, ON)
    set_digital_output(PIN_OPEN, OFF)
    set_digital_output(PIN_WIDE,  OFF)   # ★ 이 줄이 핵심(94mm 잔류 방지)
    wait(1.5)

def get_safe_vel(base_vel):
    global CURRENT_SPEED_RATIO
    return max(10, int(base_vel * CURRENT_SPEED_RATIO))

# ==========================================================
# 3. 로봇 동작 시퀀스 (test(4).py 최신 로직 반영)
# ==========================================================

# (1) Pick Filter (run_pick_filter_sequence 반영)
def sequence_pick_filter(vel=100, acc=100):
    print("=== [Sequence] PICK FILTER START ===")
    
    initialize_robot_tool("Tool Weight", "GripperDA", vel, acc)
    gripper_open()

    initial_pose = posj([6.845, 3.351, 90.331, -0.175, 84.003, -11.663])
    filter_pose_pre = posj([12.557, 38.631, 132.647, 14.701, -79.879, 89.113])
    filter_pose = posj([11.157, 40.599, 124.127, 13.852, -73.648, 87.869])
    filter_pose_up = posj([5.738, 4.463, 110.467, 17.646, -24.381, 76.336])
    filter_pose_up_post = posj([-25.228, 12.677, 88.086, 25.676, -7.735, 65.953])
    filter_pose_post = posj([-51.328, 59.663, 52.511, -84.261, 133.62, 22.864])
    filter_pose_post_post = posj([-56.895, 43.272, 66.323, -88.407, 129.105, 19.202])
    filter_pose_drop = posj([-58.595, 48.597, 70.56, -82.317, 126.982, 30.648])
    filter_pose_push_pre_pre = posj([-51.883, 40.956, 47.111, -89.938, 130.794, 0.036])
    filter_pose_push_pre = posj([-98.908, 34.083, 18.596, 1.387, 127.95, 14.356])
    filter_pose_push = posj([-99.594, 22.166, 63.827, 2.978, 95.295, 9.695])
    filter_last_first = posx([-62.688, -504.252, 590.592, 165.085, -178.44, 146.402])
    filter_last_second = posx([96.574, -448.882, 618.168, 96.111, -179.722, 95.885])

    movej(initial_pose, vel=vel, acc=acc); mwait()
    movej(filter_pose_pre, vel=vel, acc=acc); mwait()
    movej(filter_pose, vel=vel, acc=acc); mwait()
    gripper_closed()

    movej(filter_pose_up, vel=vel, acc=acc); mwait()
    movej(filter_pose_up_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_post_post, vel=vel, acc=acc); mwait()
    movej(filter_pose_drop, vel=vel, acc=acc); mwait()
    
    gripper_open()

    movej(filter_pose_push_pre_pre, vel=vel, acc=acc); mwait()
    movej(filter_pose_push_pre, vel=vel, acc=acc); mwait()
    gripper_closed()
    movej(filter_pose_push, vel=vel, acc=acc); mwait()
    movel(filter_last_first, vel=vel, acc=acc); mwait()
    movel(filter_last_second, vel=vel, acc=acc); mwait()
    
    print("=== PICK FILTER COMPLETE ===")

# (2) Scoop (run_scoop_sequence_joint 반영)
def sequence_scoop_joint(vel=50, acc=50):
    print("=== [Sequence] SCOOP START ===")
    
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", vel, acc)
    gripper_open()

    home = posj([0,0,90,0,90,0])
    scoop_approach  = posj([-57.05, 6.049, 108.19, -1.602, 62.046, 39.618])
    scoop_up        = posj([-6.989, 2.199, 89.892, 0.354, 86.657, -17.35])
    move_cup        = posx([204.3, -274.282, 681.772, 102.646, -175.475, -162.022])
    cup_up_first    = posx([-29.698, -287.555, 683.108, 101.592, -175.962, -162.972])
    cup_up_second   = posx([-34.487, -436.03, 566.438, 100.638, -174.732, -164.163])
    cup_down_first  = posx([-241.693, -488.041, 465.501, 179.75, -126.105, -102.649])
    cup_down_second = posx([-289.375, -471.722, 442.287, 178.345, -124.309, -98.711])
    cup_down_third  = posx([-262.545, -440.454, 183.676, 169.023, -57.383, -76.898])
    cup_up_return_first = posx([93.83, -332.521, 636.258, 78.345, -173.67, 163.539])
    cup_up_return_second   = posx([218.969, -362.568, 326.118, 160.762, 178.238, -109.933])

    gripper_open()
    movej(home, vel=vel, acc=acc); mwait()
    movej(scoop_up, vel=vel, acc=acc); mwait()

    movej(scoop_approach, vel=vel, acc=acc); mwait()
    gripper_closed()

    movel(move_cup, vel=vel, acc=acc); mwait()
    movel(cup_up_first, vel=vel, acc=acc); mwait()
    movel(cup_up_second, vel=vel, acc=acc); mwait()

    movel(cup_down_first, vel=vel, acc=acc); mwait()
    movel(cup_down_second, vel=vel, acc=acc); mwait()
    movel(cup_down_third, vel=vel, acc=acc); mwait()
    movel(cup_down_first, vel=vel, acc=acc); mwait()

    movel(cup_up_return_first, vel=vel, acc=acc); mwait()
    movel(cup_up_return_second, vel=vel, acc=acc); mwait()

    gripper_open()
    print("=== SCOOP COMPLETE ===")

# (3) Drip + Corn (HOT) - [run_pick_drip_spiral_sequence 반영]
def sequence_drip_corn_HOT():
    print("=== [Sequence] DRIP + CORN (HOT) START ===")
    
    # 1. DRIP
    initialize_robot_tool("Tool Weight", "GripperDA", 100, 100)
    gripper_open()

    initial_pose = [-12.547, 3.93, 89.795, 0.636, 83.999, -31.147]
    initial_pose_post =[572.688, -267.758, 407.747, 86.623, 90.988, 88.243]
    
    grip_pose_pre = [-36.048, 79.262, 31.271, 76.77, 116.035, -25.368]
    grip_pose = [-32.98, 79.395, 32.182, 76.772, 116.572, -27.1]
    grip_pose_post = posx([564.131, -265.837, 448.807, 86.319, 91.332, 87.84])
    
    drip_pose_pre = [-139.826, 23.075, 92.657, 66.953, 131.54, -38.487]
    drip_pose_1 = [-139.826, 23.074, 92.657, 66.955, 131.539, -11.237]
    drip_pose_1_2 = [-140.198, 23.896, 91.039, 68.214, 132.575, -9.092]
    drip_pose_2 = [-139.826, 23.074, 92.657, 66.955, 131.539, -5.207]
    drip_pose_2_2 = [-140.198, 23.896, 91.039, 68.214, 132.575, -5.207]
    drip_pose_3 = [-138.03, 23.541, 88.005, 71.531, 131.463, 11.786]
    drip_pose_3_2 = [-138.231, 23.401, 86.95, 73.194, 132.565, 18.227]
    drip_all = [-135.714, 24.96, 81.541, 76.933, 130.696, 34.994]
    
    drip_home_first = posx([622.525, -344.134, 214.137, 83.547, 134.566, 92.034])
    drip_home_second = posx([535.126, -296.654, 287.331, 85.341, 92.104, 87.909])

    movej(initial_pose, vel = 100, acc = 100)
    movej(initial_pose_post, vel = 100, acc = 100)
    movel(drip_home_first, vel=200, acc=200)
    movej(grip_pose, vel=100, acc=100)
    gripper_closed()
    
    print('moving...')
    movel(grip_pose_post, vel=50, acc=50)
    movej(drip_pose_pre, vel=50, acc=50)

    print("first drip....")
    movej(drip_pose_1_2, vel=50, acc=50)
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=50, acc=50)
    wait(12.0)
    
    movej(drip_pose_2_2, vel=50, acc=50)
    print("second drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_pose_pre, vel=50, acc=50)
    wait(12.0)
    
    movej(drip_pose_3_2, vel=50, acc=50)
    print("third drip....")
    move_spiral(rev=3, rmax=25, lmax=0, vel=9, acc=30, axis=DR_AXIS_Z, ref=DR_BASE)
    wait(1.0)
    movej(drip_all, vel=50, acc=50)
    movej(drip_pose_pre, vel=50, acc=50)
    movel(grip_pose_post, vel=50, acc=50)
    movej(grip_pose, vel=50, acc=50)
    wait(1.0)
    gripper_open()
    wait(1.0)
    movel(drip_home_first, vel=200, acc=200)
    movel(drip_home_second, vel=200, acc=200)
    
    movej(initial_pose, vel = 100, acc = 100)
    print("=== PICK DRIP SPIRAL SEQUENCE FINISH ===")

    # 2. CORN
    print("=== CORN SEQUENCE START (after DRIP) ===")
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)
    
    home = posj([0,0,90,0,90,0])
    corn_approach               = posx([249.466, -291.977, 415.322, 119.193, -176.125, -140.495])
    corn_move                   = posx([-252.153, -485.207, 397.362, 112.232, -176.032, -147.181])
    corn_move_2                 = posx([-244.227, -390.14, 391.839, 115.872, -176.465, -143.407])
    corn_move_gripclose_first   = posx([-350.443, -512.775, 360.696, 2.827, 112.498, 88.398])
    corn_move_gripclose_second  = posx([-339.012, -522.417, 279.587, 2.182, 104.321, 91.579])
    corn_move_gripclose_third   = posx([-256.695, -527.831, 211.872, 2.231, 96.789, 88.671])
    corn_move_third             = posx([148.621, -459.69, 246.718, 0.352, 95.692, 85.634])
    corn_move_fourth            = posx([199.966, -433.102, 158.873, 173.327, -134.387, -96.073])
    corn_move_fifth             = posx([259.322, -444.932, 121.129, 175.591, -155.109, -90.373])
    corn_move_sixth             = posx([362.302, -476.99, 344.538, 33.765, -178.573, 89.451])

    VEL = 100
    ACC = 100

    gripper_open()
    wait(5.0)
    movej(home, vel=VEL, acc=ACC); mwait()
    
    print('scoop in')
    movel(corn_approach, vel=VEL, acc=ACC); mwait()
    movel(corn_move_2, vel=VEL, acc=ACC); mwait()
    gripper_open_wide()
    movel(corn_move_gripclose_first, vel=VEL, acc=ACC); mwait()
    movel(corn_move_gripclose_second, vel=VEL, acc=ACC); mwait()
    movel(corn_move_gripclose_third, vel=VEL, acc=ACC); mwait()

    gripper_closed()

    movel(corn_move_third, vel=VEL, acc=ACC); mwait()
    movel(corn_move_fourth, vel=VEL, acc=ACC); mwait()
    movel(corn_move_fifth, vel=VEL, acc=ACC); mwait()

    gripper_open()

    movel(corn_move_sixth, vel=VEL, acc=ACC); mwait()
    movej(home, vel=VEL, acc=ACC); mwait()
    
    print("=== DRIP + CORN (HOT) COMPLETE ===")

# (4) Drip + Corn + Ice (ICE 전용 - test(4).py 반영)
def sequence_drip_corn_ICE():
    print("=== [Sequence] DRIP + CORN + ICE (ICE) START ===")
    
    # 1. Hot Drip & Corn 선행
    sequence_drip_corn_HOT()

    # 2. ICE 동작 추가 (속도 100 적용)
    print("=== ICE SEQUENCE START ===")
    
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)

    home = posj([0,0,90,0,90,0])
    scoop_approach  = posx([614.923, 207.339, 257.257, 160.743, -173.832, -112.913])
    scoop_back      = posj([22.675, 38.767, 57.022, -2.826, 87.728, 120.318])
    scoop_up        = posx([364.095, 180.291, 543.406, 10.158, 176.533, 102.411])
    ice_move_first  = posx([220.604, -344.674, 531.829, 3.904, 176.811, 96.375])
    ice_move_second = posx([-206.799, -497.996, 497.829, 4.45, 176.906, 97.276])
    ice_move_third  = posx([-170.265, -510.787, 457.536, 20.017, 176.391, 112.722])
    ice_down_first  = posx([51.115, -536.661, 406.13, 172.813, 134.785, -94.021])
    cup_up_return_first     = posx([272.998, -73.852, 441.73, 167.837, -176.627, -104.467])
    cup_up_return_second    = posx([455.164, 188.835, 364.774, 162.799, -176.761, -109.421])

    VEL = 100
    ACC = 100

    gripper_open()
    movej(home, vel=VEL, acc=ACC); mwait()

    print('scoop in (ICE)')
    movel(scoop_approach, vel=VEL, acc=ACC); mwait()

    gripper_closed()

    movej(scoop_back, vel=VEL, acc=ACC); mwait()
    movel(scoop_up, vel=VEL, acc=ACC); mwait()

    movel(ice_move_first, vel=VEL, acc=ACC); mwait()
    movel(ice_move_second, vel=VEL, acc=ACC); mwait()
    movel(ice_move_third, vel=VEL, acc=ACC); mwait()

    movel(ice_down_first, vel=VEL, acc=ACC); mwait()

    movel(ice_move_third, vel=VEL, acc=ACC); mwait()
    movel(cup_up_return_first, vel=VEL, acc=ACC); mwait()
    movel(cup_up_return_second, vel=VEL, acc=ACC); mwait()

    movej(scoop_back, vel=VEL, acc=ACC); mwait()
    movel(scoop_approach, vel=VEL, acc=ACC); mwait()
    gripper_open()
    
    print("=== ICE SEQUENCE COMPLETE ===")
    movej(home, vel=100, acc=100); mwait()

# (5) [NEW] Cup Grip Sequence - [test(4).py 최신 좌표]
def sequence_cup_grip():
    print("=== [Sequence] CUP GRIP START ===")
    
    # Tool/TCP 설정
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)

    home = posj([0,0,90,0,90,0])
    cup_approach        = posx([93.612, -293.879, 371.383, 100.71, 179.615, 100.611])
    cup_grip_zero       = posx([-49.589, -481.814, 439.56, 78.529, 179.142, 168.692])
    cup_grip            = posx([-109.21, -527.416, 431.222, 76.631, 179.208, 166.511])
    cup_grip_first      = posx([-109.655, -520.59, 338.141, 79.047, 178.993, 169.339])
    
    cup_grip_second     = posx([-48.76, -483.296, 518.162, 77.259, 179.08, 167.309])
    cup_grip_third      = posx([338.467, 15.301, 497.518, 66.844, 179.138, 156.429])
    cup_down            = posx([339.584, 16.953, 268.591, 82.64, 179.516, 172.152])
    
    # [수정됨] 상세 시퀀스 시작
    print("=== CUP GRIP SEQUENCE START ===")
    VEL = 100
    ACC = 100
    
    gripper_open()
    # 1) ice home
    movej(home, vel=VEL, acc=ACC); mwait()
    
    print('scoop in')
    movel(cup_approach, vel=VEL, acc=ACC); mwait()
    movel(cup_grip_zero, vel=VEL, acc=ACC); mwait()
    movel(cup_grip, vel=VEL, acc=ACC); mwait()
    movel(cup_grip_first, vel=VEL, acc=ACC); mwait()

    gripper_closed()

    movel(cup_grip_second, vel=VEL, acc=ACC); mwait()
    movel(cup_grip_third, vel=VEL, acc=ACC); mwait()
    movel(cup_down, vel=VEL, acc=ACC); mwait()

    gripper_open()

    movel(cup_grip_third, vel=VEL, acc=ACC); mwait()
    movej(home, vel=VEL, acc=ACC); mwait()
    
    print("=== CUP GRIP SEQUENCE COMPLETE ===")

# (6) [NEW] Topping Sequence - [test(4).py 최신 좌표 및 로직 추가]
def sequence_topping():
    print("=== [Sequence] TOPPING START ===")
    
    initialize_robot_tool("GripperDA_v1", "Tool Weight_ys", 100, 100)

    home = posj([0,0,90,0,90,0])
    topping_zero                = posx([580.649, -200.804, 527.409, 2.95, 148.087, 2.355])
    topping_first               = posx([688.477, -231.81, 359.242, 2.885, 148.543, 4.823])
    topping_second              = posx([485.769, -176.446, 467.596, 174.94, -144.454, -179.997])
    topping_third               = posx([633.565, -217.889, 443.117, 175.735, -136.125, -179.873])
    topping_move_cup_first      = posx([221.77, -6.335, 384.104, 9.943, 143.08, 3.936])
    topping_move_cup_third      = posx([162.833, 6.097, 368.872, 179.965, -126.474, 11.389])

    topping_spoon_zero          = posx([566.06, -311.677, 508.609, 3.377, 151.495, -1.955])
    topping_spoon_first         = posx([688.852, -285.248, 344.269, 177.334, -145.665, 167.994])
    topping_spoon_second        = posx([558.967, -275.751, 524.071, 176.707, -145.548, 167.634])
    topping_goto_cup_first      = posx([402.822, 18.235, 522.425, 154.771, -176.742, 116.64])
    topping_goto_cup_second     = posx([397.454, 16.661, 374.364, 146.9, -177.421, 106.815])
    
    # [NEW] test(4).py에 추가된 웨이포인트들
    go_to_first                 = posx([392.908, -88.181, 406.908, 156.217, 179.726, 155.882])
    go_to_second                = posx([392.44, -87.373, 249.228, 151.229, 179.683, 150.807])
    go_to_third                 = posx([407.737, 167.994, 265.479, 3.578, -178.459, 9.438])

    print("=== TOPPING SEQUENCE START ===")
    
    VEL = 100
    ACC = 100
    
    gripper_open()
    movej(home, vel=VEL, acc=ACC); mwait()
    
    print('scoop in')
    movel(topping_zero, vel=VEL, acc=ACC); mwait()
    movel(topping_first, vel=VEL, acc=ACC); mwait()
    gripper_closed()
    
    movel(topping_third, vel=VEL, acc=ACC); mwait()
    movel(topping_move_cup_first, vel=VEL, acc=ACC); mwait()
    movel(topping_move_cup_third, vel=VEL, acc=ACC); mwait()
    movel(topping_move_cup_first, vel=VEL, acc=ACC); mwait()
    movel(topping_third, vel=VEL, acc=ACC); mwait()
    movel(topping_first, vel=VEL, acc=ACC); mwait()
    gripper_open()

    movel(topping_spoon_zero, vel=VEL, acc=ACC); mwait()
    movel(topping_spoon_first, vel=VEL, acc=ACC); mwait()
    gripper_closed()
    movel(topping_spoon_second, vel=VEL, acc=ACC); mwait()
    movel(topping_goto_cup_first , vel=VEL, acc=ACC); mwait()
    movel(topping_goto_cup_second , vel=VEL, acc=ACC); mwait()
    
    # 나선형 동작
    move_spiral(rev=3, rmax=20, lmax=0, vel=50, acc=50, axis=DR_AXIS_Z, ref=DR_BASE)
    
    movel(topping_goto_cup_first , vel=VEL, acc=ACC); mwait()
    movel(topping_spoon_second, vel=VEL, acc=ACC); mwait()
    movel(topping_spoon_first, vel=VEL, acc=ACC); mwait()
    gripper_open()
    movej(home, vel=VEL, acc=ACC); mwait()
    
    # [NEW] 추가된 마무리 동작
    movel(go_to_first, vel=VEL, acc=ACC); mwait()
    movel(go_to_second, vel=VEL, acc=ACC); mwait()
    movel(go_to_third, vel=VEL, acc=ACC); mwait()
    movej(home, vel=VEL, acc=ACC); mwait()

    print("=== TOPPING SEQUENCE COMPLETE ===")


# ==========================================================
# 4. 작업 통합 실행 함수
# ==========================================================
def process_hot_task():
    print(">> Starting HOT Beverage Process")
    sequence_pick_filter()
    sequence_scoop_joint()
    sequence_drip_corn_HOT()
    
    sequence_cup_grip()
    sequence_topping()
    
    print(">> HOT Process DONE")

def process_ice_task():
    print(">> Starting ICE Beverage Process")
    sequence_pick_filter()
    sequence_scoop_joint()
    sequence_drip_corn_ICE()
    
    sequence_cup_grip()
    sequence_topping()
    
    print(">> ICE Process DONE")


# ==========================================================
# 5. 복구 및 제어 리스너
# ==========================================================
def recover_robot_sequence():
    print(" !!! [System] Recovery Routine Started...")
    from DSR_ROBOT2 import movej, wait
    global STOP_EVENT, EXECUTION_MODE, SAVED_STOP_INDEX, GLOBAL_STEP_COUNT, SAVED_ORDER_DATA, IS_RECOVERING
    
    IS_RECOVERING = True
    STOP_EVENT = False  
    
    try:
        db.reference(STATUS_PATH).set("RECOVERING")
    except: pass

    # 1. 하드웨어 리셋
    print("   -> 1. Resetting Hardware...")
    try:
        servo_client = DR_init.__dsr__node.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        if servo_client.service_is_ready():
            req3 = SetRobotControl.Request(); req3.robot_control = 3; servo_client.call_async(req3); time.sleep(1.0)
            req2 = SetRobotControl.Request(); req2.robot_control = 2; servo_client.call_async(req2); time.sleep(1.0)
            req1 = SetRobotControl.Request(); req1.robot_control = 1; servo_client.call_async(req1); time.sleep(1.5)
            print("      >> Servo ON.")
    except Exception as e:
        print(f"   [Warning] Servo Reset Failed: {e}")

    # 2. 소프트웨어 리셋 (모션 중지)
    print("   -> 2. Resetting Motion...")
    try:
        stop_client = DR_init.__dsr__node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
        if stop_client.service_is_ready():
            req_stop = MoveStop.Request(); req_stop.stop_mode = 2; stop_client.call_async(req_stop)
            time.sleep(0.5)
    except: pass

    # 3. 이어하기 로직
    if SAVED_STOP_INDEX > 0 and SAVED_ORDER_DATA:
        print(f"   -> 3. Resuming from Step {SAVED_STOP_INDEX}...")
        
        EXECUTION_MODE = 'RESUME' 
        GLOBAL_STEP_COUNT = 0    
        
        print("   >>> [Recovery] Switching UI to IDLE (Task Starting)")
        try:
            db.reference(STATUS_PATH).set("IDLE")
        except: pass
        
        IS_RECOVERING = False

        if SAVED_ORDER_DATA.get('temp') == 'Ice':
            process_ice_task()
        else:
            process_hot_task()
            
        EXECUTION_MODE = 'NORMAL'
        SAVED_STOP_INDEX = 0
        SAVED_ORDER_DATA = None
        print("   >>> [Recovery] Resume Completed!")
        
    else:
        print("   -> 3. No saved task. Moving Home.")
        if DR_LIB:
            home_pose = [0, 0, 90, 0, 90, 0]
            DR_LIB.movej(home_pose, vel=10, acc=10)
        print("   >>> [Recovery] Done.")
        
    IS_RECOVERING = False

# [변경됨] 별도 노드를 통해 상태 감시
def check_robot_state_polling():
    global polling_node, IS_RECOVERING
    if polling_node is None: return

    try:
        # 복구 중에는 상태 감시 무시
        if IS_RECOVERING:
            return

        # polling_node에 부착된 클라이언트 사용
        if polling_node.state_client.service_is_ready():
            req = GetRobotState.Request()
            future = polling_node.state_client.call_async(req)
            future.add_done_callback(process_state_response)
    except: pass

def process_state_response(future):
    global STOP_EVENT, SAVED_STOP_INDEX, GLOBAL_STEP_COUNT, LAST_ROBOT_STATE, IS_RECOVERING
    try:
        if IS_RECOVERING:
            return

        response = future.result()
        current_state = response.robot_state
        
        ERROR_STATES = [3, 5, 6, 7]
        
        if current_state in ERROR_STATES:
            if not STOP_EVENT:
                print(f" 🚨 [AUTO DETECT] Polling Found Error! State: {current_state}")
                STOP_EVENT = True
                SAVED_STOP_INDEX = GLOBAL_STEP_COUNT
                try:
                    db.reference(STATUS_PATH).set("EMERGENCY_STOP")
                except: pass
        
        LAST_ROBOT_STATE = current_state
    except: pass

def manage_robot_connection(command):
    global bringup_process
    if command == "CONNECT":
        if bringup_process is None:
            print(" >>> [System] Starting Robot Bringup...")
            cmd = ["ros2", "launch", "dsr_bringup2", "dsr_bringup2_rviz.launch.py", 
                   "mode:=real", "host:=192.168.1.100", "port:=12345", "model:=m0609"]
            bringup_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            print(" >>> [System] Bringup Started! (Wait 10 sec...)")
        else:
            print(" >>> [System] Robot is already connected.")
    elif command == "DISCONNECT":
        if bringup_process is not None:
            print(" >>> [System] Stopping Robot Bringup...")
            os.killpg(os.getpgid(bringup_process.pid), signal.SIGTERM)
            bringup_process = None
            print(" >>> [System] Robot Disconnected.")
        else:
            print(" >>> [System] Robot is already offline.")

def handle_speed_change(event):
    global CURRENT_SPEED_RATIO
    try:
        val = event.data
        if val is not None:
            CURRENT_SPEED_RATIO = float(val)
            speed_percent = int(CURRENT_SPEED_RATIO * 100)
            print(f" >>> [System] Real-time Speed Override: {speed_percent}%")
            if DR_LIB:
                DR_LIB.change_operation_speed(speed_percent)
    except Exception as e: 
        print(f"Speed Update Error: {e}")

def handle_order_change(event):
    global pending_order
    global STOP_EVENT
    global SAVED_ORDER_DATA, SAVED_STOP_INDEX, GLOBAL_STEP_COUNT
    
    order_data = event.data
    if order_data and isinstance(order_data, dict):
        command = order_data.get('command')
        
        if command == "START":
            STOP_EVENT = False 
            pending_order = order_data
            
            GLOBAL_STEP_COUNT = 0
            SAVED_STOP_INDEX = 0
            SAVED_ORDER_DATA = order_data
            
            print(f"[Firebase] Order: {pending_order.get('beverage')} ({pending_order.get('temp')})")
            
        elif command == "STOP":
            print(" !!! [EMERGENCY] STOP COMMAND RECEIVED !!!")
            STOP_EVENT = True 
            SAVED_STOP_INDEX = GLOBAL_STEP_COUNT 
            print(f"   -> Stopped at Step: {SAVED_STOP_INDEX}")
            
            try:
                # 긴급 정지는 제어 노드를 통해서 보냅니다 (즉시성 필요)
                stop_client = DR_init.__dsr__node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
                req = MoveStop.Request()
                req.stop_mode = 2 
                stop_client.call_async(req)
                print("   -> Stop Signal Sent.")
                db.reference(STATUS_PATH).set("EMERGENCY_STOP")
                pending_order = None 
            except Exception as e:
                print(f"Failed to call Stop Service: {e}")
            
        elif command == "RECOVER":
            print(" [Firebase] Recovery Command Received.")
            pending_order = order_data
            
        elif command == "CONNECT":
            manage_robot_connection("CONNECT")
        elif command == "DISCONNECT":
            manage_robot_connection("DISCONNECT")

def update_firebase_loop():
    while True:
        try:
            status_data = {
                "last_update_timestamp": time.time(),
                "status": "Running"
            }
            db.reference('robot_status/dsr01').update(status_data)
        except: pass
        time.sleep(0.5)

# ==========================================================
# 6. 메인 실행부 (핵심 수정됨 - Dual Node 적용)
# ==========================================================
def main(args=None):
    global pending_order
    global DR_LIB
    global polling_node 

    rclpy.init(args=args)

    # 1. 제어 노드 (Control Node) 생성
    # -> 이 노드는 오직 DR_LIB만 사용합니다. Executor에 넣지 않습니다.
    control_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = control_node

    # 2. 모니터링 노드 (Monitor Node) 생성
    # -> 이 노드는 백그라운드에서 상태를 감시합니다.
    polling_node = rclpy.create_node("dsr_monitor_node", namespace=ROBOT_ID)
    
    # 모니터링 노드에 클라이언트와 타이머 부착
    polling_node.state_client = polling_node.create_client(GetRobotState, f'/{ROBOT_ID}/system/get_robot_state')
    polling_node.create_timer(0.1, check_robot_state_polling)
    
    # 3. 백그라운드 스레드 실행 (감시 노드만 Spin)
    executor = MultiThreadedExecutor()
    executor.add_node(polling_node) # <--- 중요: control_node는 넣지 않음!
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 4. DR_LIB 로딩
    import DSR_ROBOT2 as DR_LIB_LOCAL
    DR_LIB = DR_LIB_LOCAL

    # 5. Firebase 연결
    try:
        cred = credentials.Certificate(FIREBASE_CERT_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print("[System] Firebase Connected.")
    except Exception as e:
        print(f"Firebase Init Error: {e}")
        return

    db.reference(ORDER_PATH).delete()
    db.reference(ORDER_PATH).listen(handle_order_change)
    db.reference(SPEED_PATH).listen(handle_speed_change) 
    db.reference(STATUS_PATH).set("IDLE")
    
    if db.reference(SPEED_PATH).get() is None:
        db.reference(SPEED_PATH).set(1.0)

    t_status = threading.Thread(target=update_firebase_loop)
    t_status.daemon = True
    t_status.start()

    print("==============================================")
    print("   Barista System (Dual Node Fixed)           ")
    print("==============================================")
    
    time.sleep(2.0)

    try:
        # 이제 충돌 없이 set_tool 호출 가능
        DR_LIB.set_tool("Tool Weight")
        DR_LIB.set_tcp("GripperDA")
        
        print(" >>> [Main] Waiting for orders...")
        while rclpy.ok():
            if pending_order is not None:
                current_order = pending_order
                pending_order = None 
                
                cmd_type = current_order.get('command')
                temp_type = current_order.get('temp')
                
                if cmd_type == "START":
                    try:
                        db.reference(STATUS_PATH).set("MAKING")
                        if temp_type == "Ice":
                            process_ice_task()
                        else:
                            process_hot_task()
                        
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).delete()
                        SAVED_STOP_INDEX = 0
                        
                    except Exception as e:
                        print(f"Task Stopped/Aborted: {e}")
                        db.reference(STATUS_PATH).set("ERROR")

                elif cmd_type == "RECOVER":
                    try:
                        db.reference(STATUS_PATH).set("RECOVERING")
                        recover_robot_sequence() 
                        
                        db.reference(STATUS_PATH).set("IDLE")
                        db.reference(ORDER_PATH).delete()
                    except Exception as e:
                         print(f"Recovery Error: {e}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Closing...")
    finally:
        control_node.destroy_node()
        polling_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()