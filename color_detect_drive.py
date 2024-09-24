import RPi.GPIO as GPIO
import time
import cv2
import csv
import os
from adafruit_servokit import ServoKit
import threading
import numpy as np
import pygame

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 모터 제어 클래스
class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(self.en, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 100)  # PWM 주파수 100Hz
        self.pwm.start(0)

    def set_speed(self, speed):
        self.pwm.ChangeDutyCycle(speed)

    def forward(self, speed=40):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def stop(self):
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()

# 모터 초기화
motor1 = MotorController(18, 17, 27)
motor2 = MotorController(16, 13, 26)
speed = 100
frame_time = 0.1

# PCA9685 모듈 초기화 (서보모터)
kit = ServoKit(channels=16)

# 서보모터 초기 설정
kit.servo[0].angle = 90  # 스티어링 휠 서보모터 중립 (채널 0)
kit.servo[1].angle = 85
kit.servo[2].angle = 90

# 각 색상의 HSV 범위 정의
colors = {
    "red": [(0, 120, 70), (10, 255, 255)],
    "blue": [(110, 50, 50), (130, 255, 255)],
    "yellow": [(20, 100, 100), (30, 255, 255)]
}

# 카메라 설정
cap = cv2.VideoCapture(0)
initial_color = None
waiting_time = 0.25  # 모터가 동작한 후, resume 후 대기 시간 (초)
last_frame_number = 0  # 마지막으로 읽은 프레임 번호를 저장
stop_servo_event = threading.Event()  # 서보모터를 중지하는 이벤트
resume_event = threading.Event()  # 서보모터를 재개하는 이벤트
resume_lock = threading.Lock()  # 재개 시 스레드 안전을 위한 락

# 초기 색상 감지 함수
def detect_initial_color():
    global initial_color
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:  # 일정 크기 이상일 경우
                    initial_color = color_name
                    print(f"Initial color detected: {color_name}")
                    return

# 사용자 명령을 대체할 Joystick 입력 처리 함수
def joystick_input():
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()
        if joystick.get_button(4):  # 버튼 4: 'start' 대체
            print("Motor started!")
            return 'start'
        if joystick.get_button(0):  # 버튼 0: 'resume' 대체
            print("Resuming servo operation")
            return 'resume'
        time.sleep(0.1)

# 서보 앵글 제어 함수 (CSV 파일에서 각도 불러오기)
def control_servo_from_csv(start_frame=0):
    global last_frame_number
    predicted_servo_angle_path = 'drive_data.csv'  # 예측 CSV 파일 경로

    with open(predicted_servo_angle_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # 헤더 스킵
        frame_count = 0
        for row in reader:
            with resume_lock:
                if frame_count < start_frame:
                    frame_count += 1
                    continue
                if stop_servo_event.is_set():
                    last_frame_number = frame_count
                    print(f"Servo stopped at frame {last_frame_number}")
                    return
            frame_number = int(row[0])  # 프레임 번호
            servo_angle = float(row[1])  # 서보 각도 값
            kit.servo[0].angle = servo_angle  # 서보모터에 각도 적용
            print(f"Frame {frame_count}: Servo Angle Set to {servo_angle}")
            time.sleep(frame_time)
            frame_count += 1
            
        # CSV 파일 끝에 도달하면 모터 정지
        motor1.stop()
        motor2.stop()
        last_frame_number = frame_count  # 마지막 프레임 번호 저장
        print("CSV 파일의 모든 프레임을 읽었으므로 모터가 정지됩니다.")

# 색상 감지 및 모터 제어 함수
def color_based_motor_control():
    global last_frame_number
    start_time = time.time()  # 모터가 동작한 시간을 기록

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:
                    # 모터가 시작한 후 대기 시간을 지나야 색상을 확인
                    elapsed_time = time.time() - start_time
                    if elapsed_time > waiting_time:
                        # 감지된 색상이 처음 감지한 색상과 같으면 멈춤
                        if color_name == initial_color:
                            motor1.stop()
                            motor2.stop()
                            kit.servo[0].angle = 90  # 서보모터 중립
                            stop_servo_event.set()  # 서보모터 중지 이벤트 설정
                            print(f"Initial color '{color_name}' detected again. Motors and servo stopped.")
                            return
                        else:
                            # 다른 색상이 감지되면 계속 주행
                            motor1.forward(speed)
                            motor2.forward(speed)
                            print(f"'{color_name}' detected. Motors running...")

# 메인 실행 로직
try:
    # 초기 색상 감지
    detect_initial_color()

    # Joystick 명령 대기
    while True:
        command = joystick_input()
        if command == 'start':
            # 모터 동작 시작
            motor1.forward(speed)
            motor2.forward(speed)

            # 서보모터 제어 스레드 시작
            servo_thread = threading.Thread(target=control_servo_from_csv, args=(last_frame_number,))
            servo_thread.start()

            # 색상 감지 및 모터 제어 스레드 시작
            color_thread = threading.Thread(target=color_based_motor_control)
            color_thread.start()

            # 색상 감지 스레드가 종료될 때까지 대기
            color_thread.join()

        elif command == 'resume':
            print(f"Resuming servo operation from frame {last_frame_number + 1}...")
            time.sleep(waiting_time)  # resume 후 0.5초 대기
            # 모터 다시 앞으로 움직이기
            motor1.forward(speed)
            motor2.forward(speed)
            # 서보모터 재개 이벤트 초기화
            stop_servo_event.clear()
            # 서보모터 제어 스레드 재개
            servo_thread = threading.Thread(target=control_servo_from_csv, args=(last_frame_number + 1,))
            servo_thread.start()
            # 색상 감지 및 모터 제어 스레드 재개
            color_thread = threading.Thread(target=color_based_motor_control)
            color_thread.start()
            # 색상 감지 스레드가 종료될 때까지 대기
            color_thread.join()

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    # 종료 시 모터와 GPIO 정리
    motor1.cleanup()
    motor2.cleanup()
    cap.release()
    GPIO.cleanup()
    pygame.quit()
    print("Program terminated.")