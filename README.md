# CapybaraUno
## Capybara Robot - ROS package for Indigo 
![alt text](http://i.imgur.com/QOzCvIJ.jpg "Capybara")
#### The capybara robot
the capybara robot is an open source and open hardware differential drive robot. Powered by a dspic33fj128mc802 it provides an highly customizable firmware, a full extensible serial commands stack and an high speed PID control loop up to 1khz for each motor.

The firmware source code is available https://github.com/mauriliodc/capybara

#### Ros Node
## capybarauno_solo_node
#  It subscribes to:
    cmd_vel topic to controll the robot with linear and angular velocities expressend in m/s,
    requested_ticks topic to controll the robot in ticks per second.
  It provides:
   //relative_signed_ticks topic: ticks per second expressed in signed int 16,
   //absolute_usigned_ticks: ticks per second expressed in unsigned int 16,
   //odom publish: the odometry data in messages of type nav_msgs/Odometry.
  It also broadcast the trasformation /tf from /odom to /base_link
The names of the topics can be changed with parameters. 
Can be set a robot_name and all the topics will be in the form /robot_name/topic_name.

  
