import json
import sys
import socket
from select import select


class SocketDataGetter:
    """
    This class should accept referee command data
    Fills list with json object
    """

    def __init__(self, ip="localhost", port=9999):
        self.client = self.connect(ip, port)

    def connect(self, ip, port):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((ip, port))
            print(f"<Connected to server: {ip}: {port}>")

            return client_socket

        except socket.error as msg:
            print(f"<Code: {msg.args[0]}, Error: {msg.args[1]}>")
            sys.exit()

    def main(self, out_q):
        while True:
            ready_sockets, _, _ = select([self.client], [], [], 0.01)  # 0.01 timeout
            if ready_sockets:
                data = self.client.recv(4096)
                decoded_data = data.decode("utf-8")
                json_obj = json.load(decoded_data)
                out_q.append(json_obj)


if __name__ == "__main__":
    sock = SocketDataGetter()
    q = []
    sock.main(q)
