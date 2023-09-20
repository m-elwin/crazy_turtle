# CRAZY TURTLE
Demonstration package for ME495.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the blanks marked with `${ITEM}` with your answer.
Keep the backticks but remove the `${}`, so `${ITEM}` becomes `My Answer`.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Repository Configuration
1. The `crazy_turtle` git repository consists of the ROS 2 packages `${pkg_name1}` and `${pkg_name2}`.
2. The package `${pkg_name1}` is a `${build type of package}` package.
2. The package `${pkg_name2}` is a `${build type of package}` package.


## Setup Instructions
1. Build the workspace using `${insert command here}` so that it is unnecessary to rebuild when python files change.
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing `${insert command here}`
3. Make sure no other ROS nodes are running prior to starting by inspecting the results of `${insert command here}`.
3. Run the launchfile `go_crazy_turtle.launch.xml` by executing `${insert command here}`
4. When running you can see a visual depiction of the ROS graph using the `${command}` command.
   The ROS graph, including all topics and node labels, looks like:
   ![The ROS Graph](${export svg image, add it to repository, put path here so it displays in the README.md})

## Runtime Information
The `launchfile` from above should be running at all times when executing these commands.
If the nodes launched from the `launchfile` are not running, you will get incorrect results.

5. Use the ROS command `${command and args}` to list all the nodes that are running.
   The output of the command looks like
   ```
   ${list nodes here}
   ```
6. Use the ROS command `${command and args}` to list the topics
   The output of the command looks like
   ```
   ${list topics here}
   ```

7. Use the ROS command `${command and args}` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `${frequency} Hz`

8. Use the ROS command `${command and args}` to list the services.
   The output of the command looks like
   ```
   ${list services here}
   ```

9. Use the ROS command `${command and args}` to determine the type of the `/switch` service, which is `${service type}`.

10. Use the ROS command `${command and args}` to list the parameters of all running nodes
    ```
    ${list parameters here}
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
   ${list service types here}
   ```
2. Use the ROS command `${command and args}` to list the executables included with the `crazy_turtle` package
   The output of the command looks like
   ```
   ${list executables here}
   ```

## Live Interaction
1. Use the command `${command and args here}` to retrieve the value of the `/mover velocity` parameter, which is `${value here}`.
2. The ROS command to call the `/switch` service, and it's output is listed below:
    ```
    ${enter the command and its output here. Call with x=1.0, y=2.0, theta=0.0, angular_velocity=3.0, linear_velocity=4.0}
    ```
3. The `switch` service performs the following actions (in sequence):
    1. It `${what does it do? kills | spawns | resets}` the current turtle
    2. It then respawns a new turtle at `${location as a function of the `/switch` service parameters}`
4. What happens to the turtle's motion if you use `${command and args here}` to change `/mover velocity` to 10? `${faster | slower | same}`
5. Use the Linux command `${command and args}` to kill the `/mover` node.
6. Use the ROS command `${command and args}` to start the `/mover` node with a velocity of 10. 
    - Be sure to remap `cmd_vel` to `/turtle1/cmd_vel`.
7. What happened to the turtle's velocity after relaunching `mover`? `${faster | slower | same}`
