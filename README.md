## Project in competitive robotics 2021
**Copy-paste installation:**</br>
`python3.7 -m pip install numpy`</br>
`python3.7 -m pip install opencv-python`</br>
`python3.7 -m pip install matplotlib`</br>
`pyhton3.7 -m pip install -U Pillow`</br>
`pyhton3.7 -m pip install pyrealsense2`</br>
</br>
Important folders:</br>
**STM_32** </br>
Sends bytes to motors. works in primitive way (send byte = some of motors move with some speed) </br>
**opencv_examples** </br>
Mostly contains garbage code and socket server example. sockets works, not implemented into main </br>
**threading _folder** </br>
This folder is meant  for future. It is works, but only thing is do: producer makes thresholded images, sends by queue to another thread. Consumer thread takes image and shows. That's it </br>
**main_folder** </br>
As said, main folder :). </br>
Run `python3 main.py [enable_pyrealse] [enable_calibration] [enable_gui]` in terminal.</br>
Parameters are true if input value is 1 and false, if any other integer (0 most likely).
Movement are not implemented yet
</br>
**Alghoritm:** </br>
<img src="/alghoritm.png" width=85% height="auto"/> </br>
</br>
