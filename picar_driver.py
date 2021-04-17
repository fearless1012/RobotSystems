import picarx_improved
from time import sleep

def move_forward():
    for i in range(10):
        picarx_improved.forward(80,0)
    picarx_improved.stop()

def move_backward():
    for i in range(10):
        picarx_improved.backward(80)
    picarx_improved.stop()

def move_left():
    for i in range(100):
        picarx_improved.set_dir_servo_angle(30)
    for i in range(10):
        picarx_improved.forward(40,30)
    picarx_improved.stop()

def move_right():
    for i in range(100):
        picarx_improved.set_dir_servo_angle(-30)
    for i in range(10):
        picarx_improved.forward(40,-30)
    picarx_improved.stop()

def parallel_parking(direction='l'):
    if direction == 'l':
        dir_i = -1
    else:
        dir_i=1

    picarx_improved.set_dir_servo_angle(10*dir_i)
    picarx_improved.backward(10)
    sleep(1)
    picarx_improved.set_dir_servo_angle(-10*dir_i)
    picarx_improved.forward(10, 0)
    sleep(1)
    picarx_improved.stop()

def k_turning():
    picarx_improved.set_dir_servo_angle(-10)
    picarx_improved.backward(10)
    sleep(1)
    picarx_improved.stop()
    picarx_improved.set_dir_servo_angle(20)
    picarx_improved.forward(10, 0)
    sleep(1)
    picarx_improved.stop()
    picarx_improved.forward(10, 0.60)
    sleep(1)
    picarx_improved.stop()

def keyboard_input():
    speed =0
    angle =0
    input_choice = input("used 'w-s-a-d' for 'forward-backward-left-right' maneuvering. 'e' to exit maneuvering, 'p' to park and 'k' for k_turn")

    while input_choice != 'e':
        if input_choice == 'w':
            move_forward()
        elif input_choice == 's':
            move_backward()
        elif input_choice == 'a':
            move_left()
        elif input_choice == 'd':
            move_right()
        elif input_choice == 'p':
            get_dir = input("'r' for right park and 'l' for left park")
            parallel_parking(get_dir)
            picarx_improved.stop()
        elif input_choice == 'k':
            k_turning()
            picarx_improved.stop()

        picarx_improved.set_dir_servo_angle(angle)
        picarx_improved.forward(speed)
        sleep(1)
        input_choice = input("Use 'w-s-a-d' for 'forward-backward-left-right' maneuvering. 'e' to exit maneuvering, 'p' to park and 'k' for k_turn")


if __name__ == "__main__":
    keyboard_input()
