class SocketDataGetter:
    def __init__(self):
        pass

    def main(self, out_q):
        while True:
            data = str(input(":::")) #TODO socket data
            out_q.append(data)