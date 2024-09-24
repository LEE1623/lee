import RPi.GPIO as GPIO
import pygame
import time
import csv
import os
from adafruit_servokit import ServoKit

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

    def forward(self, speed=100):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def stop(self):
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()

# 모터 초기화 (왼쪽 모터: motor1, 오른쪽 모터: motor2)
motor1 = MotorController(18, 17, 27)  # 모터1: en(18), in1(17), in2(27)
motor2 = MotorController(16, 13, 26)  # 모터2: en(16), in1(13), in2(26)

# PCA9685 모듈 초기화 (서보모터)
kit = ServoKit(channels=16)

# 서보모터 초기 설정 (스티어링 휠, 채널 0 사용)
kit.servo[0].angle = 85  # 스티어링 휠 서보모터 중립 (채널 0)

# Pygame 초기화
pygame.init()

# 조이스틱 초기화
pygame.joystick.init()

# 첫 번째 조이스틱을 선택
if pygame.joystick.get_count() == 0:
    print("조이스틱이 연결되어 있지 않습니다.")
    GPIO.cleanup()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# CSV 파일 작성 및 저장을 관리하는 변수
saving_data = False
csv_writer = None
csv_file = None
frame_count = 0  # 프레임 번호
drive_number = 0  # 드라이브 번호

# 드라이브 디렉토리 및 파일 경로를 관리하는 변수
drive_dir = ''
csv_file_path = ''

# 속도 제어 변수
speed = 0  # 속도 초기값
motor_running = False  # 모터 상태 (처음엔 정지 상태)

# CSV 파일 열기 및 저장 제어 함수
def start_saving():
    global csv_writer, csv_file, frame_count, drive_number, drive_dir, csv_file_path

    # 'data' 디렉토리가 존재하지 않으면 생성
    data_dir = 'data'
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # 기존 드라이브 디렉토리 목록 가져오기
    existing_drives = [d for d in os.listdir(data_dir) if os.path.isdir(os.path.join(data_dir, d)) and d.startswith('drive_')]

    # 드라이브 번호 추출 및 다음 번호 결정
    drive_numbers = []
    for d in existing_drives:
        try:
            num = int(d.split('_')[1])
            drive_numbers.append(num)
        except (IndexError, ValueError):
            continue

    if drive_numbers:
        drive_number = max(drive_numbers) + 1
    else:
        drive_number = 1

    # 드라이브 디렉토리 이름 생성 (예: drive_001)
    drive_dir = os.path.join(data_dir, f'drive_{drive_number:03d}')
    os.makedirs(drive_dir)

    # CSV 파일 경로 설정
    csv_file_path = os.path.join(drive_dir, 'drive_data.csv')

    # CSV 파일 열기 및 헤더 작성
    csv_file = open(csv_file_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['frame', 'servo_angle', 'speed'])  # CSV 헤더
    frame_count = 0  # 프레임 번호 초기화
    print(f"데이터 저장 시작: {csv_file_path}")

def stop_saving():
    global csv_file
    if csv_file:
        csv_file.close()
        csv_file = None
        print("데이터 저장 중단.")

# 서보모터 각도 초기값 설정
servo_angle = 85  # 스티어링 서보모터 중립
frame_time = 0.1

# 메인 루프
running = True
button_4_pressed = False  # 버튼 4가 눌렸는지 확인하는 변수
while running:
    for event in pygame.event.get():
        # 버튼 이벤트 처리
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 14:  # 버튼 14: 정지 버튼
                motor1.stop()
                motor2.stop()
                motor_running = False  # 모터 정지 상태
                saving_data = False  # 모터가 멈추면 저장도 멈춤
                stop_saving()
                print("모터 정지 및 저장 중단")

            if event.button == 0:  # 버튼 0: 저장 중단
                if saving_data:
                    saving_data = False
                    stop_saving()

            if event.button == 4:  # 버튼 4: 저장 대기 (스틱이 올라가야 저장 시작)
                button_4_pressed = True
                print("버튼 4가 눌림: 스틱이 위로 올라가면 저장을 시작합니다.")

        elif event.type == pygame.QUIT:
            running = False

    # 우측 스틱을 위로 올리면 모터가 구동됨
    axis_value_speed = joystick.get_axis(3)  # 우측 스틱 위아래 축
    if axis_value_speed < -0.1:  # 스틱을 위로 올리면 모터 작동
        if not motor_running:  # 모터가 정지 상태일 때만 실행
            speed = 100  # 모터 속도를 100으로 설정
            motor1.forward(speed)
            motor2.forward(speed)
            motor_running = True
            print("모터 구동 시작")

        # 스틱이 위로 올라가 있고, 버튼 4가 이미 눌렸으며, 모터가 작동 중일 때만 저장 시작
        if button_4_pressed and not saving_data and motor_running:
            saving_data = True
            start_saving()

    # 주행 중에도 계속 각도 조정 가능
    axis_value_steer = joystick.get_axis(0)  # 좌측 스틱 (스티어링 휠 서보모터 제어)
    servo_angle = (1 - axis_value_steer) * 90  # 각도를 0 ~ 180도 범위로 변환
    servo_angle = max(0, min(180, servo_angle))  # 각도를 0 ~ 180도로 제한
    kit.servo[0].angle = servo_angle  # 스티어링 서보모터 각도 설정

    # 데이터 저장 (저장 중일 때만)
    if saving_data and csv_writer:
        csv_writer.writerow([frame_count, servo_angle, speed if motor_running else 0])  # 프레임 번호, 서보 각도, 속도 저장
        print(f"프레임 {frame_count}, 서보 각도 {servo_angle}, 속도 {speed if motor_running else 0} 저장 완료")
        frame_count += 1

    time.sleep(frame_time)

# 종료 시 리소스 정리
motor1.cleanup()
motor2.cleanup()
GPIO.cleanup()
pygame.quit()