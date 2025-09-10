# CRAZY TURTLE
Demonstration package for ME450 Embedded Systems in Robotics.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the blanks marked with `${ITEM}` with your answer.
Keep the backticks (\`) but remove the `${}`, so `${ITEM}` becomes `My Answer`.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Repository Configuration
1. The `crazy_turtle` git repository consists of the ROS 2 packages `${pkg_name1}` and `${pkg_name2}`.
2. The package `${pkg_name1}` has `<build_type>` of `${insert <build_type>}`.
2. The package `${pkg_name2}` has a `<build_type>` of `${insert <build_type>}`.


## Setup Instructions
1. Build the workspace using `${insert command here}`.
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing `${insert command here}`
3. Make sure no other ROS nodes are running prior to starting by inspecting the results of the ROS command `${insert command here}`.
3. Run the launchfile `go_crazy_turtle.launch.xml` by executing `${insert command here}`.
4. When running you can see an interactive visual depiction of the ROS graph using the `${command}` command.
   The ROS graph, including all topics and node labels, looks like:
   ![The ROS Graph](${export svg image from the viewer, add it to your homework repository, put path here so it displays in the README.md})

## Runtime Information
The `launchfile` from above should be running at all times when executing the following commands.
If the nodes launched from the `launchfile` are not running, you will get incorrect results.

5. Use the ROS command `${command and args}` to list all the nodes that are running.
   The output of the command looks like
   ```
   ${list each node, 1 per line.}
   ```
6. Use the ROS command `${command and args}` to list the topics
   The output of the command looks like
   ```
   ${list each topic, 1 per line}
   ```

7. Use the ROS command `${command and args}` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `${frequency} Hz`

8. Use the ROS command `${command and args}` to list the services.
   The output of the command looks like
   ```
   ${list each service, 1 per line}
   ```

9. Use the ROS command `${command and args}` to determine the type of the `/switch` service, which is `${service type}`.

10. Use the ROS command `${command and args}` to list the parameters of all running nodes
    ```
    ${list each parameter here, 1 per line}
    ```

11. Use the ROS command `${command and args}` to get information about the `/mover` `velocity` parameter, including its type, description, and constraints
    ```
    ${full output of the command here}
    ```

12. Use the ROS command `${command and args}` to retrieve a template/prototype for entering parameters for the `/switch` service on the command line.
    ```
    ${full output of the command here}
    ```

## Package Exploration
1. Use the ROS command `${command and args}` to list the interface types defined by `crazy_turtle_interfaces`
   The output of the command looks like
   ```
   ${list service types here, 1 per line}
   ```
2. Use the ROS command `${command and args}` to list the executables included with the `crazy_turtle` package
   The output of the command looks like
   ```
   ${list executables here, 1 per line}
   ```

## Live Interaction
1. Use the command `${command and args here}` to retrieve the value of the `/mover velocity` parameter, which is `${value here}`.
2. The ROS command to call the `/switch` service is
    ```
    ${enter the command to clal the service with with x=1.3, y=2.1, theta=0.1, angular_velocity=3.1, linear_velocity=4.0}
    ```
3. The return value of the service is (to two decimal places): `x = ${x value} y = ${y value}`.

4. The mover node logged the following information in response to the service call:
   ```
   ${enter each logged message, 1 per line}
   ```
5. What happens to the turtle's motion if you use `${command and args here}` to change `/mover velocity` to 12 while the launchfile is running? `${faster | slower | same}`
6. Use the Linux command `${command and args}` to kill the `/mover` node.
7. Use the ROS command `${command and args}` to start the `/mover` node with a velocity of 12.
    - HINT: Be sure to remap `cmd_vel` to `/turtle1/cmd_vel`.
8. What happened to the turtle's velocity after relaunching `mover`? `${Answer faster OR slower OR same}`
