from time import time
from picarx_improved import picarx_improved

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

    def read_threaded(self, bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            bus.write(self.read())
            sleep(delay)

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

    def process_threaded(self, in_bus: Bus, out_bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            sensor_vals = in_bus.read()
            if sensor_vals is not None:
                control_val = self.process(sensor_vals)
                out_bus.write(control_val)
            sleep(delay)


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

    def control_threaded(self, bus: Bus, delay: float, kill_thread: Event):
        while not kill_thread.is_set():
            self.steer(bus.read())
            sleep(delay)

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
