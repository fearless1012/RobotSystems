from time import time
from picarx_improved import picarx_improved
from rossros import *

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
        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

    def sensor_reading(self):
        adc_output = []
        adc_output.append(self.S0.read())
        adc_output.append(self.S1.read())
        adc_output.append(self.S2.read())
        return adc_output

class Interpreter:
    def __init__(self, adc_output, sensitivity = 0.5, polarity= 0):
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.prev_output = adc_output
        self.line_detect = [False, True, False]


    def processing(self, adc_output):

        new_output = adc_output
        for ai in range(len(new_output)):
            diff = self.prev_output[ai] - new_output[ai]
            if abs(diff) > self.sensitivity:
                if diff * self.polarity < 0:
                    self.line_detect[ai] = False
                else:
                    self.line_detect[ai] = True

        self.prev_output = new_output

        if self.line_detect== [True, True, False]:
            return -0.5
        elif self.line_detect == [False, True, True]:
            return 0.5
        elif self.line_detect == [True, False, False]:
            return -1.0
        elif self.line_detect == [False, False, True]:
            return 1.0
        else:
            return 0.0

class Controller:

    def __init__(self, scale=10, speed=40):
        self.scale = scale
        self.speed = speed

    def steer(self, car, direction):
        angle = self.scale * direction
        car.set_dir_servo_angle(angle)
        car.forward(speed=self.speed, steering_angle=angle)


class camera_Sensor:

    def __init__(self):
        Vilib.camera_start(True)
        self.sensitivity = sensitivity
        self.capture = cv2.VideoCapture(0)

    def sensor_reading(self):
        r,frame = self.capture.read()
        return frame




class camera_Interpreter:
    def __init__(self):
        self.frame = frame

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


class camera_Controller:

    def __init__(self, speed = 40):
        self.speed = speed

    def steer(self, car, lanes):
        # get the steering angle
        x1, _, x2, _ = lanes[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90
        car.forward(speed=self.speed, steering_angle=steering_angle)

class ultrasonic_Sensor:
    def __init__(self):
        self.D0 = Pin("D0")
        self.D1 = Pin("D1")

    def sensor_reading(self):
        distance = Ultrasonic(pin_D0, pin_D1).read()
        return distance

class untrasonic_Interpreter:
    def __init__(self, thresh= 10):
        self.thresh = thresh
        self.stop = False

    def processing(self, distance):
        if distance < self.thresh:
            self.stop = True
        else:
            self.stop = False

        return self. stop

class ultrasonic_Controller:
    def __init__(self, speed=40):
        self.speed = speed

    def steer(self, car, stop):
        if stop:
            car.stop()
        else:
            car.forward(self.speed)

if __name__ == "__main__":
    sensitivity = 0.80
    polarity = 0
    scale = 120
    runtime = 10
    speed = 40
    thresh = 20

    car = picarx_improved()
    sensor = Sensor()
    interpreter = Interpreter(sensor.sensor_reading(), sensitivity, polarity)
    control = Controller(scale=scale, speed = speed)
    cam_sensor = camera_Sensor()
    cam_interpreter = camera_Interpreter()
    cam_control = camera_Controller(speed = speed)
    us_sensor = ultrasonic_Sensor()
    us_interpreter = untrasonic_Interpreter(thresh=thresh)
    us_control = ultrasonic_Controller(speed = speed)

    # setup busses
    #grayscale
    sensor_values_bus = Bus(initial_message=[0, 0, 0],
                            name="sensor values bus")
    sensor_interpreter_bus = Bus(initial_message=0,
                          name="sensor interpreter bus")

    #camera
    camera_values_bus = Bus(initial_message=0,
                            name="camera values bus")
    camera_interpreter_bus = Bus(initial_message=0,
                          name="camera interpreter bus")

    # ultrasonic sensor busses
    us_values_bus = Bus(initial_message=0,
                           name="ultrasonic sensor bus")
    us_interpreter_bus = Bus(initial_message=False,
                                name="ultrasonic interpreter bus")

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    # grayscale sensor threads
    greyscale_read = Producer(sensor.sensor_reading(),
                              output_busses=sensor_values_bus,
                              delay=0.09,
                              name="Greyscale Sensor Reading")

    greyscale_proc = ConsumerProducer(interpreter.processing,
                                      input_busses=sensor_values_bus,
                                      output_busses=sensor_interpreter_bus,
                                      delay=0.1,
                                      name="Greyscale Sensor Processing")
    greyscale_cont = Consumer(control.steer,
                          input_busses=sensor_interpreter_bus,
                          delay=0.1,
                          name="Greyscale Steering Controller")

    cam_read = Producer(cam_sensor.sensor_reading(),
                        output_busses=camera_values_bus,
                        delay=0.09,
                        name="Camera Sensor Reading")

    cam_proc = ConsumerProducer(cam_interpreter.processing,
                                      input_busses=camera_values_bus,
                                      output_busses=camera_interpreter_bus,
                                      delay=0.1,
                                      name="Camera Sensor Processing")
    cam_cont = Consumer(cam_control.steer,
                              input_busses=camera_interpreter_bus,
                              delay=0.1,
                              name="Camera Steering Controller")

    us_read = Producer(us_sensor.sensor_reading(),
                              output_busses=us_values_bus,
                              delay=0.09,
                              name="Ultrasonic Sensor Reading")

    us_proc = ConsumerProducer(us_interpreter.processing,
                                      input_busses=us_values_bus,
                                      output_busses=us_interpreter_bus,
                                      delay=0.1,
                                      name="Ultrasonic  Sensor Processing")
    us_cont = Consumer(us_control.steer,
                              input_busses=us_interpreter_bus,
                              delay=0.1,
                              name="Ultrasonic Steering Controller")

    thread_list = [greyscale_read, greyscale_proc, greyscale_cont, cam_read, cam_proc, cam_cont, us_read, us_proc, us_read]
    runConcurrently(thread_list)
    picar.stop()