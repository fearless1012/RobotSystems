import sys
sys.path.append(r'/opt/ezblock')
from vilib import Vilib
import cv2
import numpy as np
from picarx_improved import picarx_improved
from threading import Event, Lock
import concurrent.futures
import signal
from bus import Bus

try:
    from ezblock import *
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (/opt/ezblock is not present). Shadowing hardware calls with substitute functions")
    from sim_ezblock import *

class Sensor:

    def __init__(self):
        Vilib.camera_start(True)
        self.sensitivity = sensitivity
        self.capture = cv2.VideoCapture(0)

    def read_threaded(self, bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            bus.write(self.read())
            sleep(delay)

    def sensor_reading(self):
        r,frame = self.capture.read()
        return frame




class Interpreter:
    def __init__(self):
        self.frame = frame

    def process_threaded(self, in_bus: Bus, out_bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            sensor_vals = in_bus.read()
            if sensor_vals is not None:
                control_val = self.process(sensor_vals)
                out_bus.write(control_val)
            sleep(delay)


    def processing(self, frame):
        #detect edges
        filter_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #create mask
        min_hue = np.array([60, 40, 40])
        max_hue = np.array([150, 255, 255])
        mask = cv2.inRange(filter_img, min_hue, max_hue)
        edges = cv2.Canny(mask, 200, 400)

        #detect line segment
        segments = cv2.HoughLineSP(edges, 1, np.pi/180 , 10, np.array([]))

        #detect lanes
        lanes = []
        h,w = frame.shape
        boundary = 1 / 3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary
        for segment in segments:
            for x1, y1, x2, y2 in segment:
                if x1 ==x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lanes.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lanes.append(self.make_points(frame, right_fit_average))
        return lanes


class Controller:

    def __init__(self):
        self.scale = scale

    def control_threaded(self, bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            self.steer(bus.read())
            sleep(delay)

    def steer(self, lanes):
        # get the steering angle
        x1, _, x2, _ = lanes[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90
        return steering_angle

def run_single_thread(car, sensor, proc, control):
    max_time = 10
    car.forward(speed)
    t = time()
    while time() - t < max_time:
        vals = sensor.read()
        dir_val = proc.process(vals)
        control.steer(dir_val)
    car.stop()


def sigint_handler(sig, frame):
    global _stop_requested
    _stop_requested.set()


if __name__ == "__main__":

    sensitivity = 0.80
    polarity = 0
    scale = 120
    runtime = 10
    speed = 40

    car = picarx_improved()
    sensor = Sensor()
    interpreter = Interpreter()
    control = Controller()

    t = time()
    while time() - t < runtime:
        frame = sensor.sensor_reading()
        lanes = interpreter.processing(frame)
        angle = control.steer(lanes)
        car.forward(speed=speed, steering_angle=angle)

    # setup busses
    sensor_values_bus = Bus()
    interpreter_bus = Bus()

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor.read_threaded, sensor_values_bus,
                                  sensor_delay, _stop_requested)
        eInterpreter = executor.submit(proc.process_threaded,
                                       sensor_values_bus,
                                       interpreter_bus, interpreter_delay,
                                       _stop_requested)
        eController = executor.submit(control.steer_threaded, interpreter_bus,
                                      control_delay, _stop_requested)
        eSensor.result()



    car.stop_motors()


