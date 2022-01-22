
## Project in competitive robotics 2021
**Installation:**</br>
Move to project folder and run `python3.7 -m pip install -r requirements.txt`
Then go to the `.segment_module` folder and run `pip install .` 
Doesen't tested on other python version.</br>

**Used modules/libraries:**</br>
`Numpy` for image processing operations.</br>
`OpenCV` as optional GUI module.</br>
`Pyrealsense2` for realsense camera configuration and working with (especially getting depth).</br>
`Pyserial` for sending motor speed values (HW communication).</br>
`Websockets` for receiveng commands from referee. </br>
`Pynput` for robot manual control. </br>

**How to run the robot?:**</br>
Make sure, that you run first `image_calibration.py` file. Adjust all colors to be detected. Edit ip and port of the referee in `main.py` file, then run it. It you don't have any referees with their servers near by, run `_referee_server.py` . In server shell, you can send commands like "blue", "rose", and "stop", that will run the robot. In main shell, you can press the G to take manual control.