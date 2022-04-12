## Project in competitive robotics 2021
<img src="/images/court.svg" width=95% height="auto"/></br>

**Credits:**</br>
Programming - **Leonid Tsigrinski**</br>
Mechanics - **Veronika Podliesnova** </br>
Electronics and firmware - **Valentin Resapow** </br>

<details><summary>Photos of Robot and PCB</summary>
<img src="/images/robot.jpg" width=35% height="auto"/></br>
<img src="/images/pcb1.jpg" width=35% height="auto"/></br>
<img src="/images/pcb2.jpg" width=35% height="auto"/></br>
</details>


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

**Game logic:**  
Basically, there are two threads (asyncio coroutines), socket listener thread listens to to referee JSON data and fills global queue variable with it. Game Logic thread firstly checks if manual control enabled. Then checks if global queue variable were filled with commands. If it's a start command (with target basket), then the robot reads the camera image, gets the depth of the basked as well as coordinates of the basket and the balls. Then the robot selects the closest ball and moves to it, and for one iteration enables state machine algorithm. Every iteration state remains remembered. State machine by itself, it's just few steps:

-   find a ball (basically rotate)
-   get to a ball (calculate distance between ball x, y coords and the center coords. Distance is in linear dependency with axes speed)
-   find a basket (rotate around the wall until basket and the ball isn't centered)
-   throw (calculate throwing speed depends on the distance, linear dependency as well)

<img src="/images/block_diagram.png" width=100% height="auto"/></br>
