import time
import cv2
import numpy as np
import RTIMU
import serial
from datetime import datetime
from picamera import PiCamera
from astropy.time import Time
from adafruit_servokit import ServoKit


class GPSModule:
    def __init__(self, port="/dev/ttyS0", baudrate=9600, timeout=1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def get_time(self):
        try:
            while True:
                line = self.serial.readline().decode('ascii', errors='replace').strip()
                if line.startswith("$GPRMC"):
                    parts = line.split(",")
                    if parts[2] == "A":
                        utc_time = parts[1]
                        date = parts[9]
                        time_str = f"{date[:2]}-{date[2:4]}-{date[4:6]} {utc_time[:2]}:{utc_time[2:4]}:{utc_time[4:6]}"
                        return datetime.strptime(time_str, "%d-%m-%y %H:%M:%S")
        except Exception as e:
            print(f"GPS Error: {e}")
        return None


class IMUModule:
    def __init__(self, settings_file="RTIMULib"):
        self.settings = RTIMU.Settings(settings_file)
        self.imu = RTIMU.RTIMU(self.settings)
        if not self.imu.IMUInit():
            raise Exception("IMU Initialization Failed!")
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

    def read_data(self):
        if self.imu.IMURead():
            data = self.imu.getIMUData()
            return {
                "orientation": data["fusionPose"],
                "acceleration": data["accel"],
                "gyro": data["gyro"],
            }
        return None


class ServoController:
    def __init__(self, channels=16):
        self.kit = ServoKit(channels=channels)
        self.pan_servo = self.kit.servo[0]
        self.tilt_servo = self.kit.servo[1]
        self.reset_position()

    def reset_position(self):
        self.pan_servo.angle = 90
        self.tilt_servo.angle = 90

    def move_to(self, pan_angle, tilt_angle):
        try:
            self.pan_servo.angle = max(0, min(180, pan_angle))
            self.tilt_servo.angle = max(0, min(180, tilt_angle))
        except Exception as e:
            print(f"Servo Error: {e}")


class StarTracker:
    def __init__(self, resolution=(1024, 768)):
        self.camera = PiCamera()
        self.camera.resolution = resolution

    def capture_image(self, filename="star_image.jpg"):
        self.camera.capture(filename)
        return filename

    def process_image(self, image_path):
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        _, thresh = cv2.threshold(image, 50, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        stars = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w * h > 5:
                stars.append((x + w // 2, y + h // 2))
        return stars

    def match_stars(self, stars):
        if not stars:
            return None
        print(f"Detected stars: {stars}")
        return "Orion Belt (Example Match)"


class AstroinertialNavigation:
    def __init__(self):
        self.gps = GPSModule()
        self.imu = IMUModule()
        self.servo = ServoController()
        self.star_tracker = StarTracker()
        self.log_file = "navigation_log.csv"

    def log_data(self, timestamp, imu_data, celestial_match):
        try:
            with open(self.log_file, "a") as log:
                log.write(f"{timestamp},{imu_data['orientation']},{celestial_match}\n")
        except Exception as e:
            print(f"Logging Error: {e}")

    def run(self):
        try:
            while True:
                print("---- Astroinertial Navigation System ----")
                current_time = self.gps.get_time()
                if not current_time:
                    print("GPS Time Unavailable. Retrying...")
                    continue

                print(f"Current Time: {current_time}")
                for pan in range(45, 136, 15):
                    for tilt in range(45, 136, 15):
                        self.servo.move_to(pan, tilt)
                        time.sleep(0.5)
                        image_file = self.star_tracker.capture_image()
                        stars = self.star_tracker.process_image(image_file)
                        celestial_match = self.star_tracker.match_stars(stars)
                        if celestial_match:
                            print(f"Celestial Match: {celestial_match}")
                            break

                imu_data = self.imu.read_data()
                if imu_data:
                    print(f"IMU Data: Orientation={imu_data['orientation']} Acceleration={imu_data['acceleration']} Gyro={imu_data['gyro']}")
                    self.log_data(current_time, imu_data, celestial_match)
                else:
                    print("IMU Data Unavailable.")
                time.sleep(5)
        except KeyboardInterrupt:
            print("System Stopped")
        except Exception as e:
            print(f"Unexpected Error: {e}")


if __name__ == "__main__":
    navigation_system = AstroinertialNavigation()
    navigation_system.run()
