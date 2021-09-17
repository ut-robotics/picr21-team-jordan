import socket
import sys
import threading
import time
from select import select

<<<<<<< HEAD
=======

>>>>>>> ee52732560bc392f1106fbd230ee6d85a4cb470a
try:
    port = sys.argv[0]
except IndexError:
    port = 9999

# 192.168.43.238/24 
ip = ""

try:
    socket_server = socket.socket()
    socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_server.bind((ip, port))
    socket_server.listen(5)
    print("<Server {}: {} is running.>".format(ip, port))

except socket.error as msg:
    print("<Code: {}, Error: {}>".format(msg.args[0], msg.args[1]))
    sys.exit()


def accept_data(client, connection):
    global inactive_tasks

    client_ip = connection[0]
    client_port = connection[1]
    last_task, last_state, delay = None, None, 0

    print("<{}: {} Connected.>".format(client_ip, client_port))

    while True:
        ready_sockets, _, _ = select([client], [], [], 0.01)  # timeout

        if ready_sockets:
            data = client.recv(4096)
            decoded_data = data.decode("utf-8")

            if decoded_data == "exit":
                break
            if decoded_data == "?":
                client.send("Available tasks: {}".format(inactive_tasks).encode("utf-8"))

            print("({}: {}): {}".format(client_ip, client_port, decoded_data))
            message = decoded_data.split()

            if message[0] in ["task1", "task2"] and len(message) == 3:
                if message[0] in inactive_tasks:
                    index = inactive_tasks.index(message[0])
                    message[0] = inactive_tasks.pop(index)

                    last_task, last_state, delay = message[0], message[1], message[2]
                    led_control(message[0], message[1], message[2])
                    client.send("Doing task {}. State: {}".format(message[0], message[1]).encode("utf-8"))
                else:
                    if message[0] == last_task:
                        last_state, delay = message[1], message[2]
                        client.send(
                            "Doing task {}. State: {}. Delay: {}".format(message[0], message[1], message[2]).encode(
                                "utf-8"))
                    else:
                        client.send("Task is not available.".encode("utf-8"))
            else:
                if decoded_data != "?":
                    client.send(data)
        else:
            led_control(last_task, last_state, delay)

    print("<{}: {} Disconnected.>".format(client_ip, client_port))
    client.close()
    if message[0]:
        inactive_tasks.append(message[0])


number = 0
def led_control(task, state, delay):
    global tasks, active_tasks, number, pin_led, pin_matrix, d
    time.sleep(int(delay))

    if task == "task1":
        if state == "stop":
            pass

        elif state == "eraldi":
            print("doing something 2")
            
        elif state == "koos":
            print("doing something 3")

    elif task == "task2":
        if state == "stop":
            pass

        elif state == "from_0":
            print("doing something 3")

        elif state == "from_9":
            print("doing something 4")

        elif state == "random":
            print("doing something 5")

while True:
    try:
        client, ip = socket_server.accept()
        accepter = threading.Thread(target=accept_data, args=(client, ip))
        accepter.start()
        
    except socket.error as msg:
        print("<Code: {}, Error: {}>".format(msg.args[0], msg.args[1]))
        sys.exit()
