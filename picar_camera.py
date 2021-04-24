import sys
sys.path.append(r'/opt/ezblock')
from vilib import Vilib

Vilib.camera_start(True)
Vilib.color_detect_switch(True)
Vilib.detect_color_name('red')

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

    def __init__(self, scale=10):
        self.scale = scale

    def steer(self, car, direction):
        angle = self.scale * direction
        car.set_dir_servo_angle(angle)
        return angle


if __name__ == "__main__":

    sensitivity = 0.80
    polarity = 0
    scale = 120
    runtime = 10
    speed = 40

    car = picarx_improved()
    sensor = Sensor()
    interpreter = Interpreter(sensor.sensor_reading(), sensitivity, polarity)
    control = Controller(scale=scale)

    t = time()
    while time() - t < runtime:
        readings = sensor.sensor_reading()
        direction = interpreter.processing(readings)
        angle = control.steer(car, direction=direction)
        car.forward(speed=speed, steering_angle=angle)
    car.stop_motors()


