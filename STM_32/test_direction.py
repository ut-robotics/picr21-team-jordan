import time
import math
import numpy as np
import pandas as pd


goal_dir_angle = np.linspace(0, 360, num=36)
speed_limit = 30


speed1 = np.zeros(len(goal_dir_angle))
speed2 = np.zeros(len(goal_dir_angle))
speed3 = np.zeros(len(goal_dir_angle))

print(goal_dir_angle)
for i, angle in enumerate(goal_dir_angle):
    speed1[i] = int(math.sin((angle+120)*(2*math.pi/360))*speed_limit)
    speed2[i] = int(math.sin((angle)*(2*math.pi/360))*speed_limit)
    speed3[i] = int(math.sin((angle-120)*(2*math.pi/360))*speed_limit)
    print(np.round(angle, 2), "M1",
          speed1[i], "M2", speed2[i], "M3", speed3[i])

print(speed1, "speed1")
print(speed2, "speed2")
print(speed3, "speed3")
data_mp = (np.array([goal_dir_angle, speed1, speed2, speed3])).T

data = pd.DataFrame(data_mp, columns=['angle', 'M1', 'M2', 'M3'])
data
