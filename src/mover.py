#!/usr/bin/env python
""" Publishes twist that will move a robot back and forth in the x direction """

import rospy
from geometry_msgs.msg import Twist

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
    twist.angular.z = 0
    return twist
    
class Mover(object):
    """ Puplishes movement geometry_msgs/Twist commands at a fixed rate 
    """
    def __init__(self):
      self.pub = rospy.Publisher('drive', Twist, queue_size = 10)
      self.tmr = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
      self.nsteps = 0
      self.direction = 1

    def timer_callback(self, event):
        """ Handle the timer callback.  event is the TimerEvent """
        twist = turtle_twist(self.direction, 0)

        self.nsteps += 1
        if self.nsteps > 20:
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
