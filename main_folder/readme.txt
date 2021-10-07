Hi
If you want to run test the robot with your PC, fist of all run the image_calibration.py file and make sure, that only ball is visible (or is the biggest blob at least)
Now, quit by pressing Q and run main.py
You can see every blob has three digits (red ones and green ones). This is (X:Y:::SIZE). Biggest ball should be maked as red digits.
Now you can move the ball and see that robot can udestand, does ball is on thhe centre of the frame or not.
Only thing, that you  need to edit is run_current_state() function, that contains actual action.
Uncomment marked lines when you are ready to test motors.
