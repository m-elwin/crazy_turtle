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

from enum import Enum, auto
from crazy_turtle_interfaces.srv import Switch
from geometry_msgs.msg import Twist, Vector3
from math import pi
from random import uniform
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from rcl_interfaces.msg import ParameterDescriptor


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
    return Twist(linear = Vector3(x = xdot, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = omega))

class Mover(Node):
    """ Publishes movement geometry_msgs/Twist commands at a fixed rate
    """

    def __init__(self):
        super().__init__("mover")
        self.state = State.HURTLING

        self.declare_parameter("velocity", 1.0,
                               ParameterDescriptor(description="The velocity of the turtle"))
        self.nsteps    = 0
        self.direction = 1
        self.velocity  = self.get_parameter("velocity").get_parameter_value().double_value
        self.pub       = self.create_publisher(Twist, "cmd_vel", 10)
        self.switch    = self.create_service(Switch, "switch", self.switch_callback)
        self.kill      = self.create_client(Kill, "kill")
        self.spawn     = self.create_client(Spawn,"spawn")
        self.timer     = self.create_timer(0.008333, self.timer_callback)

        if not self.kill.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "kill" service to become available')

        if not self.spawn.wait_for_service(timeout_sec=1.0):
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
        self.get_logger().info("Killing turtle1")
        self.kill_future = self.kill.call_async(Kill.Request(name="turtle1"))

        self.state = State.KILLING

        # The new position of the turtle is intentionally scrambled from a weird message
        self.newx = request.mixer.y + request.mixer.angular_velocity
        self.newy = request.mixer.x * request.mixer.linear_velocity

        response.x = self.newx
        response.y = self.newy

        return response

    def timer_callback(self):
        """ Hurtle the turtle
        """
        if self.state == State.HURTLING:
            twist = turtle_twist(self.direction * self.velocity, uniform(-20.0, 20.0))

            self.nsteps += 1
            if self.nsteps > 200:
                self.nsteps = 0
                self.direction *= -1

            self.pub.publish(twist)

        elif self.state == State.KILLING:
            if self.kill_future.done():
                self.get_logger().info("turtle1 is dead :(")
                self.get_logger().info("respawning turtle1")
                self.spawn_future = self.spawn.call_async(Spawn.Request(x = self.newx, y = self.newy, theta = uniform(-pi, pi), name = "turtle1"))
                self.state = State.SPAWNING

        elif self.state == State.SPAWNING:
            if self.spawn_future.done():
                self.state = State.HURTLING
                self.get_logger().info("turtle1 lives!")

        else:
            raise RuntimeException("Invalid State")

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    mymove = Mover()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
