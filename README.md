# Mobile-Robot

Autonomous Mobile Robot
Mini Project for Mobile_Robotics

Author : Jogesh S Nanda

Description : Controlles a 2 wheel differential drive robot to find a target space
      also uses obstracle avoidance to find a collision free path


Function :

============ functions ====================

# motor_A_encoder ---> from the interupt function increment Variable " encoder_A_pos " each time it's called
# motor_B_encoder ---> from the interupt function increment Variable " encoder_B_pos " each time it's called

# distance_motor_A ---> Global Variable which gives the distance travelled by the Left Wheel
# distance_motor_B ---> Global Variable which gives the distance travelled by the Right Wheel

# distance_reset ---> reset the encoder counter

# robot_distance ---> gives the distance travelled by the robot during forward or reverse direction

# robot_move ---> move the robot at distance X [in cm]
# robot_turn_left ---> turn the robot at angle (anti-clockwise)
# robot_turn_right ---> turn the robot at an anlge (clock-wise)
# robot_stop ---> emergency STOP

============= End =========================
