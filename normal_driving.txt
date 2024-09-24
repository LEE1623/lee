import RPi.GPIO as GPIO
import time
import csv
from adafruit_servokit import ServoKit

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 모터 제어 클래스 (이전 코드와 동일)
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

# 모터 초기화
motor1 = MotorController(18, 17, 27)
motor2 = MotorController(16, 13, 26)

# PCA9685 모듈 초기화 (서보모터)
kit = ServoKit(channels=16)

# 서보모터 초기 설정
kit.servo[0].angle = 90  # 스티어링 휠 서보모터 중립 (채널 0)
kit.servo[1].angle = 85
kit.servo[2].angle = 90

speed = 100

# 모터를 앞으로 움직이기
motor1.forward(speed)
motor2.forward(speed)

# drive_data.csv 파일에서 각 프레임당 servo_angle 값을 불러오기
drive_data_path = 'drive_data.csv'  # 새로운 CSV 파일 경로

with open(drive_data_path, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # 헤더 스킵
    frame_count = 0
    for row in reader:
        frame_number = int(row[0])  # 프레임 번호
        servo_angle = float(row[1])  # 서보 각도 값
        kit.servo[0].angle = servo_angle  # 서보모터에 각도 적용
        print(f"Frame {frame_number}: Servo Angle Set to {servo_angle}")
        time.sleep(0.1)  # 0.1초 대기 (프레임당 0.1초)
        frame_count += 1

# CSV 파일 끝까지 읽은 후 모터 정지
motor1.stop()
motor2.stop()

# 동작 종료 시 모터와 GPIO 정리
motor1.cleanup()
motor2.cleanup()
GPIO.cleanup()