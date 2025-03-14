#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。
作者: Robert 时间: 2024/07/20

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。

此模块通过话题的发布者，将双臂复合机器人模块的基本功能都进行了测试。
"""

import rospy
import json
import math
import os
import numpy as np
import time
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Pose 
from dual_arm_msgs.msg import MoveJ_P, MoveJ, Hand_Angle, Hand_Speed
from pydub import AudioSegment
import simpleaudio as sa

# Globalne zmienne dla publisherów
pub_MoveJ_l = None
pub_MoveJ_r = None
pub_hand_speed_l = None
pub_hand_angle_l = None
pub_hand_speed_r = None
pub_hand_angle_r = None
pub_mouthCloseAndOpen = None
pub_setEyelidsBlink = None

def hand_left_open(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [1000, 1000, 1000, 1000, 1000, 1000]
    pub_hand_speed_l.publish(msg_speed)
    pub_hand_angle_l.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def hand_left_close(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [0, 0, 0, 0, 1000, 1000]
    pub_hand_speed_l.publish(msg_speed)
    pub_hand_angle_l.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def hand_right_open(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [1000, 1000, 1000, 1000, 1000, 1000]
    pub_hand_speed_r.publish(msg_speed)
    pub_hand_angle_r.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def hand_right_close(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [0, 0, 0, 0, 1000, 1000]
    pub_hand_speed_r.publish(msg_speed)
    pub_hand_angle_r.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def mata_hand_right(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [100, 1000, 1000, 1000, 100, 100]
    pub_hand_speed_r.publish(msg_speed)
    pub_hand_angle_r.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def mata_hand_left(speed):
    msg = Hand_Angle()
    msg_speed = Hand_Speed()
    msg_speed.hand_speed = speed
    msg.hand_angle = [100, 1000, 1000, 1000, 100, 100]
    pub_hand_speed_l.publish(msg_speed)
    pub_hand_angle_l.publish(msg)
    rospy.loginfo("Published hand angles: %s", msg.hand_angle)

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)

def left_arm():
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.4
    movej_cmd_l.joint = [degrees_to_radians(109), degrees_to_radians(48), degrees_to_radians(43), degrees_to_radians(-174), degrees_to_radians(-76), degrees_to_radians(2)]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo(f"Publishing left_arm at {rospy.Time.now().to_sec()}")

def left_arm2():
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.4
    movej_cmd_l.joint = [degrees_to_radians(78), degrees_to_radians(50), degrees_to_radians(58), degrees_to_radians(-175), degrees_to_radians(-68), degrees_to_radians(2)]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo(f"Publishing left_arm2 at {rospy.Time.now().to_sec()}")

def right_arm():
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.4
    movej_cmd_r.joint = [degrees_to_radians(61), degrees_to_radians(56), degrees_to_radians(44), degrees_to_radians(-174), degrees_to_radians(-66), degrees_to_radians(-93)]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo(f"Publishing right_arm at {rospy.Time.now().to_sec()}")

def right_arm2():
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.4
    movej_cmd_r.joint = [degrees_to_radians(90), degrees_to_radians(45), degrees_to_radians(45), degrees_to_radians(-171), degrees_to_radians(-81), degrees_to_radians(-70)]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo(f"Publishing right_arm2 at {rospy.Time.now().to_sec()}")

def left_arm_mata():
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.4
    movej_cmd_l.joint = [degrees_to_radians(108), degrees_to_radians(91), degrees_to_radians(80), degrees_to_radians(112), degrees_to_radians(-93), degrees_to_radians(6)]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo(f"Publishing left_arm_mata at {rospy.Time.now().to_sec()}")

def left_arm_mata2():
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.4
    movej_cmd_l.joint = [degrees_to_radians(108), degrees_to_radians(75), degrees_to_radians(93), degrees_to_radians(113), degrees_to_radians(-107), degrees_to_radians(6)]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo(f"Publishing left_arm_mata2 at {rospy.Time.now().to_sec()}")

def right_arm_mata():
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.4
    movej_cmd_r.joint = [degrees_to_radians(-108), degrees_to_radians(-80), degrees_to_radians(-94), degrees_to_radians(-107), degrees_to_radians(90), degrees_to_radians(96)]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo(f"Publishing right_arm_mata at {rospy.Time.now().to_sec()}")

def right_arm_mata2():
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.4
    movej_cmd_r.joint = [degrees_to_radians(-107), degrees_to_radians(-80), degrees_to_radians(-87), degrees_to_radians(-108), degrees_to_radians(114), degrees_to_radians(96)]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo(f"Publishing right_arm_mata2 at {rospy.Time.now().to_sec()}")

def setEyelidsBlink(state):
    msg = Bool()
    msg.data = state
    pub_setEyelidsBlink.publish(msg)
    rospy.loginfo(f"Publishing setEyelidsBlink at {rospy.Time.now().to_sec()}")

def mouthCloseAndOpen(state):
    msg = Bool()
    msg.data = state
    pub_mouthCloseAndOpen.publish(msg)
    rospy.loginfo(f"Publishing mouthCloseAndOpen at {rospy.Time.now().to_sec()}")

# Definicja zdarzeń w czasie jako słownik
# Klucz: czas (float), Wartość: lista funkcji do wykonania (krotka: funkcja, argumenty)
robot_actions = {
    0.0: [(mata_hand_left, (500,)), (mata_hand_right, (500,))],
    5.0: [(hand_left_close, (100,)), (hand_right_close, (900,))],
    6.0: [(hand_left_open, (100,)), (hand_right_open, (900,))],
    7.0: [(hand_left_close, (100,)), (hand_right_close, (900,))],
    8.0: [(mata_hand_left, (500,)), (mata_hand_right, (500,)), (setEyelidsBlink, (False,)), (mouthCloseAndOpen, (True,))],
    13.0: [(left_arm, ()), (right_arm, ())],
    18.0: [(left_arm2, ()), (right_arm2, ())],
    20.0: [(left_arm_mata, ()), (right_arm_mata, ()), (setEyelidsBlink, (True,)), (mouthCloseAndOpen, (False,))]
}

# Funkcja do synchronizacji audio i ruchów robota z użyciem słownika zdarzeń
def play_audio_and_control_robot():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    generated_folder = os.path.join(script_dir, './lips_control/generate_servo_setup/generated')
    audio_folder = os.path.join(script_dir, './audio')
    vector_file_path = os.path.join(generated_folder, 'servo_angles.txt')
    time_file_path = os.path.join(generated_folder, 'timestamps.txt')

    # Znalezienie pliku audio
    wav_files = [f for f in os.listdir(audio_folder) if f.endswith('.wav')]
    if not wav_files:
        raise FileNotFoundError("Brak plików .wav w folderze 'audio'")

    wav_path = os.path.join(audio_folder, wav_files[0])

    # Sprawdzenie, czy pliki istnieją
    if not os.path.exists(vector_file_path) or not os.path.exists(time_file_path):
        raise FileNotFoundError(f"Brak plików: {vector_file_path} lub {time_file_path}")

    # Wczytanie danych
    angles = np.loadtxt(vector_file_path)
    times = np.loadtxt(time_file_path)

    if len(angles) != len(times):
        raise ValueError("Liczba kątów i czasów musi się zgadzać!")

    # Wczytanie i przygotowanie audio
    audio = AudioSegment.from_wav(wav_path)
    audio = audio.set_sample_width(2)  # Konwersja na 16-bit
    start_ms = 0  # Możesz dostosować startowy czas
    audio = audio[start_ms:]

    # Odtwarzanie audio
    play_obj = sa.play_buffer(
        audio.raw_data,
        num_channels=audio.channels,
        bytes_per_sample=audio.sample_width,
        sample_rate=audio.frame_rate
    )

    # Synchronizacja ruchów robota z audio
    start_time = time.time()
    executed_events = set()  # Zbiór do śledzenia wykonanych zdarzeń

    for i, (t, angle) in enumerate(zip(times, angles)):
        expected_time = t
        sleep_time = expected_time - (time.time() - start_time)

        if sleep_time > 0:
            time.sleep(sleep_time)

        # Sprawdzenie, czy dany czas odpowiada zdarzeniu w słowniku
        if t in robot_actions and t not in executed_events:
            for action, args in robot_actions[t]:
                action(*args)  # Wykonanie każdej funkcji z jej argumentami
            executed_events.add(t)  # Oznaczenie zdarzenia jako wykonanego

        if angle > 80:
            mouthCloseAndOpen(True)
            rospy.loginfo(f"Czas: {t:.2f} s, singing{angle:.2f}")
        rospy.loginfo(f"Czas: {t:.2f} s, Wykonano ruch: {angle:.2f}")

    play_obj.wait_done()
    
# Callback dla timera
def timer_callback(event):
    current_time = rospy.Time.now().to_sec()
    rospy.loginfo(f"Timer triggered at {current_time}")
    # Wywołanie synchronizacji audio i ruchów robota w wybranym momencie
    play_audio_and_control_robot()

def main():
    # Inicjalizacja noda ROS
    rospy.init_node('navigation_publisher', anonymous=True)

    # Inicjalizacja publisherów jako globalne zmienne
    global pub_MoveJ_l, pub_MoveJ_r, pub_hand_speed_l, pub_hand_angle_l, pub_hand_speed_r, pub_hand_angle_r, pub_mouthCloseAndOpen, pub_setEyelidsBlink
    pub_MoveJ_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_MoveJ_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_hand_speed_l = rospy.Publisher('/l_arm/rm_driver/Hand_SetSpeed', Hand_Speed, queue_size=10)
    pub_hand_angle_l = rospy.Publisher('/l_arm/rm_driver/Hand_SetAngle', Hand_Angle, queue_size=10)
    pub_hand_speed_r = rospy.Publisher('/r_arm/rm_driver/Hand_SetSpeed', Hand_Speed, queue_size=10)
    pub_hand_angle_r = rospy.Publisher('/r_arm/rm_driver/Hand_SetAngle', Hand_Angle, queue_size=10)
    pub_mouthCloseAndOpen = rospy.Publisher('/mouth_close_and_open', Bool, queue_size=10)
    pub_setEyelidsBlink = rospy.Publisher('/set_eye_lids_blink', Bool, queue_size=10)

    rospy.sleep(1)  # Czekaj na inicjalizację publisherów

    # Ustawienie timera - np. wyzwalanie co 30 sekund
    rospy.Timer(rospy.Duration(1), timer_callback)

    # Utrzymanie noda aktywnego
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass