# Fall Prevention System - ESP32 Code

from machine import Pin, I2C, UART
import time
import bluetooth
import ujson
from mpu6050 import MPU6050  # Accelerometer & Gyroscope
from pulse_sensor import PulseSensor  # Heart Rate Sensor
from edge_impulse_classifier import classify_fall  # AI Model for Fall Detection
from micropyGPS import MicropyGPS  # GPS Module

# Initialize sensors
i2c = I2C(scl=Pin(22), sda=Pin(21))
mpu = MPU6050(i2c)
heart_sensor = PulseSensor(Pin(34))

gps_uart = UART(1, baudrate=9600, tx=17, rx=16)  # GPS Module
my_gps = MicropyGPS()

# Bluetooth Setup
bt = bluetooth.Bluetooth()
bt.active(True)

# Emergency Contact Numbers
CARETAKER_PHONE = "+1234567890"  # Replace with actual caretaker number
EMERGENCY_PHONE = "+9876543210"  # Replace with nearest healthcare center

# Thresholds
HEART_RATE_THRESHOLD = 40  # bpm (Too low may indicate a problem)
GYROSCOPE_THRESHOLD = 300  # degrees/sec (Sudden movement may indicate a fall)

def read_gps():
    """Reads GPS data and returns coordinates."""
    while gps_uart.any():
        my_gps.update(gps_uart.read().decode('utf-8'))
    return my_gps.latitude, my_gps.longitude


def send_bluetooth_alert(location):
    alert_msg = ujson.dumps({
        "alert": "FALL_DETECTED",
        "location": location
    })
    bt.write(alert_msg)
    print("Bluetooth alert sent: ", alert_msg)


def make_emergency_calls():
    print(f"Calling caretaker at {CARETAKER_PHONE}...")
    time.sleep(5)  # Simulate call duration
    print(f"If no response, calling emergency services at {EMERGENCY_PHONE}...")
    time.sleep(5)

# Main Loop
while True:
    accel_data = mpu.get_acceleration()
    gyro_data = mpu.get_rotation()
    heart_rate = heart_sensor.get_heart_rate()
    gps_location = read_gps()
    
    # Combine sensor data
    sensor_data = {
        "acceleration": accel_data,
        "rotation": gyro_data,
        "heart_rate": heart_rate
    }
    
    # Predict fall using AI model
    fall_detected = classify_fall(sensor_data)
    
    # Additional conditions for alert
    if fall_detected or heart_rate < HEART_RATE_THRESHOLD or abs(gyro_data[0]) > GYROSCOPE_THRESHOLD:
        print("Fall or abnormal condition detected! Sending alert...")
        send_bluetooth_alert(gps_location)
        make_emergency_calls()
        time.sleep(10)  # Avoid repeated alerts
    
    time.sleep(1)
