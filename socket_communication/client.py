import socket
import sys
import threading

try:
    ip = sys.argv[1]
    port = int(sys.argv[2])
except IndexError:
    ip = "localhost"
    port = 1228

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))
    print("<Connected to server: {}: {}>".format(ip, port))

except socket.error as msg:
    print("<Code: {}, Error: {}>".format(msg.args[0], msg.args[1]))
    sys.exit()


def accept_response(client_socket):
    server_response = client_socket.recv(4096)
    print("Server response: {}".format(server_response.decode()))


while True:
    data = input("::: ")
    client_socket.send(data.encode('utf-8'))

    if data == 'exit':
        print("<Client disconnected>")
        break

    response_getter = threading.Thread(target=accept_response, args=(client_socket,))
    response_getter.start()
