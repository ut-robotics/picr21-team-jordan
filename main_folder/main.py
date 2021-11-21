import threading
import sys

from image_getter import *
from socket_data_getter import *
from state_machine import *

# print("python3 main.py [enable_pyrealsense][enable_gui]")
# try:
#     enable_gui = True if int(sys.argv[1]) == 1 else False
# except IndexError:
#     print("1 terminal parameters is required.")
#     sys.exit()
# except ValueError:
#     print("Terminal parameter should be int() type")
#     sys.exit()


def producer(out_q):
    camera_image = SocketDataGetter()
    camera_image.main(out_q)


def consumer(in_q):
    state_machine = ImageGetter(enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    q = []
    t1 = threading.Thread(target=producer, args=(q,))
    t1.daemon = True
    t2 = threading.Thread(target=consumer, args=(q,))
    t1.start()
    t2.start()
