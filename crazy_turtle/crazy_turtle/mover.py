# Copy this file into your homework repository and fill in the ${?} blanks in the document strings
"""
Publishes twist that will move a robot back and forth in the ${?} direction
while randomly providing a ${?}[fill in previous ${?} with either linear or angular] velocity about the ${?}-axis.

PUBLISHERS:
  + ${topic_name} (${message_type}) - The velocity of an erratic turtle path

SERVICES:
  + ${topic_name} (${service_type}) - Position of the new turtle

PARAMETERS:
  + ${param_name} (${param_type}) - Velocity driving the robot

"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from random import uniform
from crazy_turtle_interfaces.srv import Switch
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from enum import Enum, auto

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    KILLING = auto(),
    SPAWNING = auto(),
    HURTLING = auto()

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
        self.declare_parameter("velocity")

        self.nsteps    = 0
        self.direction = 1
        self.velocity  = self.get_parameter("velocity").get_parameter_value().double_value
        self.pub       = self.create_publisher("cmd_vel", Twist, queue_size = 10)
        self.switch    = self.create_service(Switch, "switch", self.switch_callback)
        self.kill      = self.create_client(Kill, "kill")
        self.spawn     = self.create_client(Spawn,"spawn")
        self.timer     = self.create_timer(0.008333, self.timer_callback)

        if not self.kill_cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "kill" service to become available')

        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "spawn" service to become available')

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
        self.kill_future = self.kill.call_async(self.kill.Request(name="turtle1"))

        self.state == KILLING

        # The new position of the turtle is intentionally scrambled from a weird message
        self.newx = request.mixer.y + request.mixer.angular_velocity
        self.newy = request.mixer.x * request.mixer.linear_velocity

        response.x = newx
        response.y = newy

        return response

    def timer_callback(self):
        """ Hurtle the turtle
        """
        if self.state == HURTLING:
            twist = turtle_twist(self.direction * self.velocity, uniform(-20, 20))

            self.nsteps += 1
            if self.nsteps > 200:
                self.nsteps = 0
                self.direction *= -1

            self.pub.publish(twist)

        elif self.state == KILLING:
            if self.kill_future.done:
                self.spawn_future = self.spawn(self.spawn.Request(x = newx, y = newy, theta = uniform(-pi, pi), name = "turtle1"))
                self.state = SPAWNING

        elif self.state == SPAWNING:
            if self.spawn_future.done:
                self.state = HURTLING

        else:
            except RuntimeException("Invalid State")

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    mymove = Mover()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
