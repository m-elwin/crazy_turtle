# Copy this file into your homework repository and fill in the ${?} blanks in the document strings
"""
Publishes twist that will move a robot back and forth in the ${?} direction
while randomly providing a ${?}[fill in previous ${?} with either linear or angular] velocity about the ${?}-axis.

PUBLISHERS:
  + ${topic_name} (${message_type}) ~ the velocity of an erratic turtle path

SERVICES:
  + ${topic_name} (${service_type}) ~ position of the new turtle

"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
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
    return Twist(linear = Vector3(x = xdot, y = 0, z = 0),
                  angular = Vector3(x = 0, y = 0, z = omega))

class Mover(Node):
    """ Publishes movement geometry_msgs/Twist commands at a fixed rate
    """

    def __init__(self):
        super().__init__("mover")
        self.nsteps = 0
        self.direction = 1
        self.velocity = rospy.get_param("~velocity")
        self.pub = self.create_publisher("cmd_vel", Twist, queue_size = 10)
        self.switch = self.create_service(Switch, "switch", self.switch_callback)
        self.kill = rospy.ServiceProxy("kill", Kill)
        self.spawn = rospy.ServiceProxy("spawn", Spawn)
        self.timer = self.create_timer(0.008333, self.timer_callback)


    def switch_callback(self, request, response):
        """ Callback function for the ${?} service

        Kills turtle1 and respawns it an a new location

         Args:
          request (SwitchRequest): the mixer field contains
             x, y, linear and angular velocity components
             that are used to determine the new turtle location

          response (SwitchResponse): the response object

        Returns:
           A SwitchResponse, containing the new x and y position
        """
        self.kill("turtle1")

        # The new position of the turtle is intentionally scrambled from a weird message
        newx = pose.mixer.x * pose.mixer.angular_velocity
        newy = pose.mixer.y * pose.mixer.linear_velocity
        self.spawn(x = newx, y = newy, theta = pose.mixer.theta, name = "turtle1")
        return SwitchResponse(x = newx, y = newy)

    def timer_callback(self):
        """ Hurtle the turtle
        """
        twist = turtle_twist(self.direction * self.velocity, uniform(-20, 20))

        self.nsteps += 1
        if self.nsteps > 200:
            self.nsteps = 0
            self.direction *= -1

        self.pub.publish(twist)

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    mymove = Mover()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
