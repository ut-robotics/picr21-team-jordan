from threading import Thread


def get_sum(name, iterations):
    for _ in range(iterations):
        print(name)
        

t1 = Thread(target=get_sum, args=("t1", 99))
t2 = Thread(target=get_sum, args=("t2", 99))
t3 = Thread(target=get_sum, args=("t3", 99))
t1.start()
t2.start()
t3.start()

t1.join()
t2.join()
t3.join()

print("----------------------------------done----------------------------------")