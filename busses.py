from time import time


class Bus:

    def __init__(self, msg_type=None):
        self.type = msg_type
        self.message = None
        self.time = None

    def read(self):
        return self.message

    def write(self, msg):

        if self.type is not None:
            assert type(msg) is self.type,\
                "Message must be of type {}".format(self._type)

        self.message = msg
        self.time = time()


if __name__ == "__main__":

    # create bus with msg type of int
    bus = Bus(int)

    # write message to bus
    msg = 10
    bus.write(msg)

    # read message from bus
    msg, t = bus.read()
    print("Message: {}\nTimestamp: {}".format(msg, t))