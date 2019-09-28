#!/usr/bin/env python
""" 
Publishes twist that will move a robot back and forth in the x direction 

PUBLISHERS:
  + cmd_vel (geometry_msgs/Twist) ~ the velocity of an erratic turtle path

SERVICES:
  + switch (geometry_msgs/Pose) ~ position of the new turtle

"""

import rospy
from geometry_msgs.msg import Twist, Pose2D
from random import uniform
from crazy_turtle.srv import Switch, SwitchResponse
from turtlesim.srv import Spawn, SpawnRequest
from turtlesim.srv import Kill

def turtle_twist(xdot, omega):
    """ Create a twist suitable for a turtle

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    twist = Twist()
    twist.linear.x = xdot
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = omega
    return twist

class Mover(object):
    """ Puplishes movement geometry_msgs/Twist commands at a fixed rate 
    """
    def __init__(self):
        self.nsteps = 0
        self.direction = 5
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.switch = rospy.Service("switch", Switch, self.switch_callback)
        self.kill = rospy.ServiceProxy("kill", Kill)
        self.spawn = rospy.ServiceProxy("spawn", Spawn)
        self.tmr = rospy.Timer(rospy.Duration(0.01), self.timer_callback)


    def switch_callback(self, pose):
        self.kill("turtle1")
        # The new position of the turtle is intentionally scrambled from a weird message
        newx = pose.mixed_up.x * pose.mixed_up.angular_velocity
        newy = pose.mixed_up.y * pose.mixed_up.linear_velocity
        self.spawn(x = newx, y = newy, theta = pose.mixed_up.theta, name = "turtle1")
        return SwitchResponse(x = newx, y = newy)

    def timer_callback(self, event):
        """ Handle the timer callback.  event is the TimerEvent """
        twist = turtle_twist(self.direction, uniform(-10, 10))

        self.nsteps += 1
        if self.nsteps > 200:
            self.nsteps = 0
            self.direction *= -1
            self.pub.publish(twist)

def main():
    rospy.init_node('mover')
    mymove = Mover()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
