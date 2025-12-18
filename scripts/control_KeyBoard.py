#!/usr/bin/python3
import rospy
from pynput import keyboard, mouse
from crt_driver.msg import crt_encoder  # 左右轮速度消息

import time

# ================= 键盘状态 =================
key_state = {'w': False, 's': False, 'a': False, 'd': False}
turn_value = 0
last_move_time = time.time()
last_x = None

target_left = 0
target_right = 0
current_left = 0
current_right = 0

# ================= 参数 =================
SPD_FWD = 120
SPD_BACK = -120
SPEED_STEP = 4   # 平滑控制步长
TURN_GAIN = -0   # 鼠标灵敏度
KEY_TURN = 70    # A/D 转向增量
MAX_SPEED = 150
MIN_SPEED = -150

# ================= 控制逻辑 =================
def update_targets():
    global turn_value, target_left, target_right
    forward = 0
    if key_state['w']:
        forward = SPD_FWD
    elif key_state['s']:
        forward = SPD_BACK

    turn = turn_value
    if key_state['a']:
        turn += KEY_TURN
    if key_state['d']:
        turn -= KEY_TURN

    left = int(forward - turn)
    right = int(forward + turn)
    # 限幅
    target_left = max(MIN_SPEED, min(MAX_SPEED, left))
    target_right = max(MIN_SPEED, min(MAX_SPEED, right))

# ================= ROS 发布 =================
def publish_speed(pub):
    global current_left, current_right
    # 平滑控制
    if current_left < target_left:
        current_left = min(target_left, current_left + SPEED_STEP)
    elif current_left > target_left:
        current_left = max(target_left, current_left - SPEED_STEP)

    if current_right < target_right:
        current_right = min(target_right, current_right + SPEED_STEP)
    elif current_right > target_right:
        current_right = max(target_right, current_right - SPEED_STEP)

    msg = crt_encoder()
    msg.leftEncoder = int(current_left)
    msg.rightEncoder = int(current_right)
    pub.publish(msg)

# ================= 键盘事件 =================
def on_press(key):
    try:
        k = key.char.lower()
        if k == 'q':
            rospy.signal_shutdown("用户退出")
            return False
        if k in key_state:
            key_state[k] = True
            update_targets()
    except:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in key_state:
            key_state[k] = False
            update_targets()
    except:
        pass

# ================= 鼠标事件 =================
def on_move(x, y):
    global last_x, turn_value, last_move_time
    if last_x is None:
        last_x = x
        return
    dx = x - last_x
    last_x = x
    turn_value = dx * TURN_GAIN
    last_move_time = time.time()
    update_targets()

# ================= 主程序 =================
if __name__ == "__main__":
    rospy.init_node("keyboard_encoder_control")
    pub = rospy.Publisher("/encoder_toMCU", crt_encoder, queue_size=10)

    print("------------------------------")
    print(" FPS 模式遥控机器人（ROS版）")
    print(" W/S：前进 后退")
    print(" A/D：键盘转向")
    print(" 鼠标左右：转向")
    print(" Q：退出并停车")
    print("------------------------------")

    kb_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    kb_listener.start()
    mouse_listener = mouse.Listener(on_move=on_move)
    mouse_listener.start()

    rate = rospy.Rate(50)  # 50Hz 控制频率
    while not rospy.is_shutdown():
        # 鼠标停止自动回正
        if time.time() - last_move_time > 0.1:
            if turn_value != 0:
                turn_value = 0
                update_targets()
        publish_speed(pub)
        rate.sleep()

