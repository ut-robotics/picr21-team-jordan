## Project in competitive robotics 2021
**Compatibility:**</br>
`Python 3.7.x` but any one can work as well, since we don't use `pyrealsense`</br>
`cv2 4.5.3`</br>
`numpy 1.21.2`</br>
`imutils 0.5.4`</br>

Important folders:</br>
**STM_32** </br>
Sends bytes to motors. works in primitive way (send byte = some of motors move with some speed) </br>
**opencv_examples** </br>
Mostly contains garbage code and socket server example. sockets works, not implemented into main </br>
**threading _folder** </br>
This folder is meant  for future. It is works, but only thing is do: producer makes thresholded images, sends by queue to another thread. Consumer thread takes image and shows. That's it </br>
**main_folder** </br>
As said, main folder :). There's also readme in detail inside, but in two words: just run calibration, play with sliders and adjust the mask image. Then run the main code. Main code can currently understand, does ball in center or not, but movement is not implemented yet. This is our main task now -- implement movement (move forward, backward, left, right) </br>
</br>
**Alghoritm:** </br>
<img src="/alghoritm.png" width=85% height="auto"/> </br>
</br>
