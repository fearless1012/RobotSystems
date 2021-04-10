class Servo():
    def __init__(self, pwm):
      return

    def angle(self, angle):
        return

class PWM():
    def __init__(self, channel, debug="critical"):
        return

    def i2c_write(self, reg, value):
        return

    def freq(self, *freq):
        return

    def prescaler(self, *prescaler):
        return

    def period(self, *arr):
        return

    def pulse_width(self, *pulse_width):
        return

    def pulse_width_percent(self, *pulse_width_percent):
        return

class Pin():
    def __init__(self, *value):
        return

    def check_board_type(self):
        return

    def init(self, mode):
        return

    def dict(self, *_dict):
        return

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        return

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        return

    def name(self):
        return "GPIO%s" % self._pin

    def names(self):
        return [self.name, self._board_name]

    class cpu(object):
        def __init__(self):
            pass

class ADC():
    def __init__(self, chn):
        return

    def recv(self, recv, addr=0x00, timeout=0):
        return recv

    def read(self):
        value_h = self.recv(1, self.ADDR)[0]
        value_l = self.recv(1, self.ADDR)[0]
        value = (value_h << 8) + value_l
        return value

    def read_voltage(self):
        return self.read * 3.3 / 4095







