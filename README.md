# tell_your_motor_speed
Repo for motor speed control using PID control amd speech recognition .
Project by Abdallah Hesham :)
## This project implements speed control of a DC motor using PID control and Arduino, and adds the ability to choose one of several setpoints for the speed value using your voice!
### The speech recognition model is built in Tensorflow, I basically followed the tutorial in their documentation, and used the public speech commands dataset(link is in the end with the citation)

### To allow communication from python file to Arduino, I used ROS, so to use this package, you will need to install rosserial_python, which you'll find details on how to install it here: https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros - automatic!
[here] (https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros) 
## Arduino applies PID control on the motor and publishes the speed in an angular.x twist msg, and the current setpoint in the angular.y field for the same msg and 
## there is a separate topic for the current PID output, to change the setpoint run the speech recognition python file using '<roslaunch tell_your_motor_speed launcher.launch>' 
## You will see an image asking you to get ready, it will last for 1 second, then an image that indicates 'recording' will show, you need to time your command with the  ## second image, commands available are digits from 0-9 and 'stop', digits will choose a certain speed but 'stop' will set the setpoint to 0
![get ready](images.jpeg)
   
Format: ![source] [https://cutt.ly/icMoriR]
   
   
![recording](index.png)

Format: ![source] [https://droplr.com/how-to/productivity-tools/picking-the-best-audio-recording-software-for-your-computer/]


## You need to upload the code to Arduino, run roscore, then run <rosrun rosserial_python serial_node.py /dev/ttyACM0> Then you are ready to go.
### if you get stuck when running the command '<rosrun rosserial_python serial_node.py /dev/ttyACM0>' check which port your Arduino is connected to from the IDE.
### You will need to have sounddevice, scipy, TensorFlow and rospy to use this project



### For this project I used a DC motor with an encoder, and an L298 H-bridge, one encoder output pin is connected to Arduino's digital pin 2, in1 and in2 in the ##L298 are connected to pins 6,7 in the Arduino, and PWM pin 9 in the Arduino is connected to enable pin in the H-bridge. 





Dataset citation:
@article{speechcommandsv2,
   author = { {Warden}, P.},
    title = "{Speech Commands: A Dataset for Limited-Vocabulary Speech Recognition}",
  journal = {ArXiv e-prints},
  archivePrefix = "arXiv",
  eprint = {1804.03209},
  primaryClass = "cs.CL",
  keywords = {Computer Science - Computation and Language, Computer Science - Human-Computer Interaction},
    year = 2018,
    month = apr,
    url = {https://arxiv.org/abs/1804.03209},
}



