# CRAZY TURTLE
Demonstration package for ME495.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the the blanks marked with `<angle brackets>` with your answer.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Setup Instructions
1. Compile the workspace by executing `<insert command here>`
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing `<insert command here>`
3. Run the launchfile `<location of launch file>` by executing `<insert command here>`
4. When running you can see a visual depiction of the ROS graph using the `<command>` command.
   The ROS graph, including all topics and node labels, looks like:
   ![<The ROS Graph>](<path_to_image_here_include_image_in_your_repository>)

## Runtime Information
The `launchfile` from above should be running when executing these commands.

5. Use the ROS command `<command and args>` to list all the nodes that are running.
   The output of the command looks like
   ```
   <list nodes here>
   ```
6. Use the ROS command `<command and args>` to list the topics
   The output of the command looks like
   ```
   <list topics here>
   ```

7. Use the ROS command `<command and args>` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `<frequency> Hz`

8. Use the ROS command `<command and args>` to list the services.
   The output of the command looks like
   ```
   <list services here>
   ```
9. Use the ROS command `<command and args>` to view information about the `/switch` service.
   The type of the `/switch` service is `<service type>` and it is offered by
   the `<name of node>` node.

10. Use the ROS command `<command and args>` to list the parameters that are loaded
    into the parameter server.
    The output of the command looks like
    ```
    <list parameters here>
    ```

## Package and Dependencies
11. Use the ROS command `<command and args>` to list the immediate (direct) dependencies of `crazy_turtle`
   The output of the command looks like
   ```
   <list direct dependencies here>
   ```
12. Use the ROS command `<command and args>` to list the types of services defined by `crazy_turtle`
    The output of the command looks like
    ```
    <list service types here>
    ```
## Live Interaction
13. Use the ROS command `<command and args>` to call the `/switch` service.
    The command returns `<return value>` and the turtle <brief description of what the turtle does>.
    (Hint: use `rossrv info` on the type of the `/switch` service to see the parameters.
     To test the behavior, look at the code or try calling with `x = 1`, `y = 1`, once with `linear_velocity = 0` and `angular_velocity = 0` and once with these at different nonzero values.)
14. What is the value of the `/mover/velocity` parameter? <value here>
15. What happens to the turtle's motion if you change `/mover/velocity` to 10 while the turtlesim and mover node are running? <faster | slower | same>
16. Use the ROS command `<command and args>` to kill the `/mover` node.
17. Use the ROS command `<command and args>` to start the `/mover` node. Be sure to
    remap `cmd_vel` to `/turtle1/cmd_vel`.
18. What happened to the turtle's velocity after relaunching `mover`? <faster | slower | same>