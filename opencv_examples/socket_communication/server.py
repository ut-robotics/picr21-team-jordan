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
    socket_server.listen(5)
    print(f"<Server {ip}: {port} is running.>")

except socket.error as msg:
    print(f"<Code: {msg.args[0]}, Error: {msg.args[1]}>")
    sys.exit()


def accept_data(client, connection):
    client_ip = connection[0]
    client_port = connection[1]
    print(f"<{client_ip}: {client_port} Connected.>")

    while True:
        # select == Wait for I/O
        ready_sockets, _, _ = select([client], [], [], 0.01)  # 0.01 timeout

        if ready_sockets:
            data = client.recv(4096)
            decoded_data = data.decode("utf-8")
            print(f"({client_ip}: {client_port}): {decoded_data}")
            # message = decoded_data.split()
            client.send("Test response from server".encode("utf-8"))

            if decoded_data == "exit":
                break

    print("<{client_ip}: {client_port} Disconnected.>")
    client.close()


while True:
    try:
        client, ip = socket_server.accept()
        accepter = threading.Thread(target=accept_data, args=(client, ip))
        accepter.start()

    except socket.error as msg:
        print(f"<Code: {msg.args[0]}, Error: {msg.args[1]}>")
        sys.exit()
