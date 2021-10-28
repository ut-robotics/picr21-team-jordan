## Project in competitive robotics 2021
**Copy-paste installation:**</br>
`python3.7 -m pip install numpy`</br>
`python3.7 -m pip install opencv-python`</br>
`python3.7 -m pip install matplotlib`</br>
`pyhton3.7 -m pip install -U Pillow`</br>
`pyhton3.7 -m pip install pyrealsense2`</br>
</br>
Important files in main folder:</br>
`image_calibration.py`: only adjust threshold values and save them into `/config`</br>
`image_getter.py`: gets coordinates of the ball and of the basket. Sends them to the `state_machine.py`</br>
`state_machine.py`: mechanical level robot controling. Actual movement code.</br>
`socket_data_getter.py`: gets the commands from referees.</br>
**`main.py`**: creates 2 parralel threads. `image_getter.py` and `socket_data_getter.py`. Communication using same list.</br>
**Alghoritm:** </br>
<img src="/alghoritm.png" width=85% height="auto"/> </br>
</br>
