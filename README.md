# Mobile-Robot

Autonomous Mobile Robot
Mini Project for Mobile_Robotics

Author : Jogesh S Nanda

Description : Controlles a 2 wheel differential drive robot to find a target space
      also uses obstracle avoidance to find a collision free path

==========================================================================
# Table of contents
==========================================================================
1. Introduction
2. Design



==========================================================================
# 1. Introduction
==========================================================================

  A robot which can move from one point to other is called a mobile robot. Due to advancement
in technology and industrial need to adapt to new changes in manufacturing methods and logistics
robots are in huge demand. Mobile robots are widely used in logistics and material handling to cater
faster and efficient movement of materials. These mobile robots which work autonomously deliver
the product from one location to other. Algorithm from higher hierarchy transmits only the
destination coordinates to these mobile robots.

  The mini-project done here is to get a hands on experience in building an autonomous mobile
robot which can suit this demand. Differential drive robot is designed, build and tested in real world
condition into which we give the position (x, y) coordinates to reach the destination avoiding
unperceived obstacle.

==========================================================================
# 2. Design
==========================================================================

# Aim/ problem statement:
  Drive the mobile robot to a specific goal location given by (X, Y) co-ordinates through
Bluetooth. Robot has to move to this specific goal location avoiding obstacles.

# Approach:
  Goal location is reached by moving the robot in its axis at each time. Robot is first moved
along y axis then its turned 90 degree left or right and then moved through x axis.
