import picarx_improved
from picarx_improved import *

def move_forward():
    for i in range(1000):
        picarx_improved.forward(10,0)
    picarx_improved.stop()

def move_backward():
    for i in range(1000):
        picarx_improved.backward(10)
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
    picarx_improved.set_dir_servo_angle(10 * dir_i)
    picarx_improved.backward(10)
    sleep(1)
    picarx_improved.stop()
    picarx_improved.set_dir_servo_angle(-20 * dir_i)
    picarx_improved.forward(10, 0)
    sleep(1)
    picarx_improved.stop()
    picarx_improved.forward(10, 0.60)
    sleep(1)
    picarx_improved.stop()

def keyboard_input():
    speed =0
    angle =0
    input_choice = get_char("used 'w-s-a-d' for 'forward-backward-left-right' maneuvering. 'e' to exit maneuvering, 'p' to park and 'k' for k_turn")

    while input_choice != 'e':
        if input_choice == 'w':
            speed +=10
        elif input_choice == 's':
            speed -= 10
        elif input_choice == 'a':
            angle += 5
        elif input_choice == 'd':
            angle -= 5
        elif input_choice == 'p':
            get_dir = get_char("'r' for right park and 'l' for left park")
            parallel_parking(get_dir)
            picarx_improved.stop()
        elif input_choice == 'k':
            k_turning()
            picarx_improved.stop()

        picarx_improved.set_dir_servo_angle(angle)
        picarx_improved.forward(speed)
        input_choice = get_char("Use 'w-s-a-d' for 'forward-backward-left-right' maneuvering. 'e' to exit maneuvering, 'p' to park and 'k' for k_turn")


if __name__ == "__main__":
    try:
        while 1:
            keyboard_input()
    finally:
        stop()