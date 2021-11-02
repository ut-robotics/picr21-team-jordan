p = {"USB VID:PID=0483:5740 SER=207738905056 LOCATION=1-3:1.0": "ttyACM0", "1":"2","3":"2", "4":"2"}
for key in p.keys():
    print(key, 228)
    if "USB VID:PID=0483:5740" in key:
        print(p[key])
        break