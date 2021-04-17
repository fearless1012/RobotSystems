
import time
import logging
import atexit
import math
from logdecorator import log_on_start, log_on_end, log_on_error

logging_format = "%( asctime)s: %( message)s"
logging.basicConfig(level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

try:
    from ezblock import *
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (/opt/ezblock is not present). Shadowing hardware calls with substitute functions")
    from sim_ezblock import *

class picarx_improved:
    def __init__(self):
        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.02

        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")

        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        self.Servo_dir_flag = 1
        self.dir_cal_value = 0
        self.cam_cal_value_1 = 0
        self.cam_cal_value_2 = 0
        self.motor_direction_pins = [left_rear_dir_pin, right_rear_dir_pin]
        self.motor_speed_pins = [left_rear_pwm_pin, right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)


    @log_on_start(logging.DEBUG , "stopping motors")
    @log_on_error(logging.DEBUG , "error stopping motors")
    @log_on_end(logging.DEBUG , "stopped motors successfully")
    def stop_motors(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def set_motor_speed(self, motor, speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # if speed != 0:
        #     speed = int(speed / 2) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)


    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0


    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]


    def dir_servo_angle_calibration(self, value):
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
        # dir_servo_pin.angle(dir_cal_value)


    def set_dir_servo_angle(self, value):
        self.dir_servo_pin.angle(value + self.dir_cal_value)


    def camera_servo1_angle_calibration(self, value):
        self.cam_cal_value_1 = value
        self.set_camera_servo1_angle(self.cam_cal_value_1)
        # camera_servo_pin1.angle(cam_cal_value)


    def camera_servo2_angle_calibration(self, value):
        self.cam_cal_value_2 = value
        self.set_camera_servo2_angle(self.cam_cal_value_2)
        # camera_servo_pin2.angle(cam_cal_value)


    def set_camera_servo1_angle(self, value):
        self.camera_servo_pin1.angle(-1 * (value + self.cam_cal_value_1))


    def set_camera_servo2_angle(self, value):
        self.camera_servo_pin2.angle(-1 * (value + self.cam_cal_value_2))


    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(S0.read())
        adc_value_list.append(S1.read())
        adc_value_list.append(S2.read())
        return adc_value_list


    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)


    def backward(self, speed):
        logging.debug("backward")
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)


    def forward(self, speed, steering_angle = 0):
        logging.debug("forward")
        car_length = 10
        car_width = 11
        radius =  car_length/math.cos(steering_angle)
        wheelr_1 = radius + (car_width/2)
        wheelr_2 = radius - (car_width/2)

        speed_1 = -1*(speed*radius/wheelr_1)
        speed_2 = -1*(speed*radius/wheelr_2)
        print(speed_1)
        print(speed_2)
        self.set_motor_speed(1, speed_1)
        self.set_motor_speed(2, speed_2)


    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)


    def Get_distance(self):
        timeout = 0.01
        trig = ezblock.Pin('D8')
        echo = ezblock.Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        # print(cm)
        return cm


def test(picarx_improved):
    picarx_improved.dir_servo_angle_calibration(-10)
    picarx_improved.set_dir_servo_angle(-40)
    time.sleep(1)
    picarx_improved.set_dir_servo_angle(0)
    time.sleep(1)
    picarx_improved.set_motor_speed(1, 1)
    picarx_improved.set_motor_speed(2, 1)
    atexit.register(stop_motors)
    # camera_servo_pin.angle(0)

if __name__ == "__main__":
    picar = picarx_improved()
    try:
        picar.dir_servo_angle_calibration(-10)
        while 1:
            test(picar)
    finally:
        picar.stop()
