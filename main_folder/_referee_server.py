import socket
import sys
import threading
from select import select

try:
    port = int(sys.argv[1])
except IndexError:
    port = 9999

# 192.168.43.238/24
ip = ""

try:
    socket_server = socket.socket()
    socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_server.bind((ip, port))
    socket_server.listen(1)
    print(f"<Server {ip}: {port} is running.>")

except socket.error as msg:
    print(f"<Code: {msg.args[0]}, Error: {msg.args[1]}>")
    sys.exit()


def accept_data(client, connection):
    client_ip = connection[0]
    client_port = connection[1]
    print(f"<{client_ip}: {client_port} Connected.>")
    print("Available commands: blue, rose, stop, fake. Just type them in")
    while True:
        # select == Wait for I/O
        # ready_sockets, _, _ = select([client], [], [], 0.01)  # 0.01 timeout
        start_blue = '{"signal": "start","targets":  ["001TRT"],"baskets": ["blue"]}'
        start_rose = '{"signal": "start","targets":  ["001TRT"],"baskets": ["magneta"]}'
        stop = '{"signal": "stop", "targets":  ["001TRT"]}'
        fake = '{"signal": "start","targets":  ["any_id_whatever"],"baskets": ["magneta"]}'
        input_data = input()
        if input_data == "blue":
            client.send(start_blue.encode("utf8"))
        elif input_data == "rose":
            client.send(start_rose.encode("utf8"))
        elif input_data == "stop":
            client.send(stop.encode("utf8"))
        elif input_data == "fake":
            client.send(fake.encode("utf8"))
        


while True:
    try:
        client, ip = socket_server.accept()
        accepter = threading.Thread(target=accept_data, args=(client, ip))
        accepter.start()

    except socket.error as msg:
        print(f"<Code: {msg.args[0]}, Error: {msg.args[1]}>")
        sys.exit()
