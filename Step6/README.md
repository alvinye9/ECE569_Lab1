# Step 6: Creating a ROS Package for rrbot
Until now, we have placed URDF/xacro files along with our launch files into a single folder. This is not the recommended way to organize our `.xacro` files. There is a standard way to package a robot description into a ROS package. In this tutorial, we will learn how to create our own ROS package for our rrbot.

## Creating a Description Package with RosTeamWS
Follow the instructions below to create an rrbot description package.

First, we must create a new package in our existing workspace, `ws1`. Source the workspace with `_ws1` and then change into the `src` folder with `rosds`. Create a new package like so:
```bash
_ws1
rosds
create-new-package rrbot_description "a description package for my rrbot"
```
Follow the prompts as follows:
```
(1) standard
(2) global git
(4) Apache-2.0
(1) ament_cmake
(ENTER)
(no) setup/update repository
```
*Note: the `ament_python` doesn't seem to work with the`setup-robot-description` script that we will run later, so we will use `ament_cmake` for the description package. Don't worry, we won't be writing any C++ code.*

Then build the package:
```bash
rosd
colcon build --symlink-install --packages-select rrbot_description
```

You should verify that the `rrbot_description` package builds and files have been added to the `src/rrbot_description` folder.

Now we are ready to turn `rrbot_description` into a description package. 
```bash
cd src/rrbot_description
setup-robot-description rrbot
```
and follow the prompts:
```
(2) python launch files
(ENTER)
```

Now build and source the workspace again:
```bash
rosd
colcon build --symlink-install --packages-select rrbot_description
_ws1
```

Now we should be able to launch the `view_rrbot.launch.py` script:
```bash
ros2 launch rrbot_description view_rrbot.launch.py
```
The robot shown is a default robot, and will look nothing like our rrbot, but we can quickly fix that!

## Fixing rrbot
Navigate to `ws1/src/rrbot_description/urdf`. Then you can delete the `rrbot/rrbot_macro.ros2_control.xacro` file, since we won't be using ros2 control here. Then, replace the `rrbot/rrbot_macro.xacro` and `rrbot.urdf.xacro` files with the starter code provided in the Step6 folder. Be sure to not move the locations of the files within the `ws1/src/rrbot_description/urdf` folder.

There are some slight differences between the starter code provided in Step 6 and then code provided in Step 5. Let's start with the `rrbot.urdf.xacro` file. 

First, we see that a xacro arg is used. 
```xml
<xacro:arg name="prefix" default="" />
```
These arguments are passed into the `xacro` tool in the `view_rrbot.launch.py` file. Xacro arguments are different than xacro properties/params because a property/param is set within the `.xacro` file itself. The way arguments are used is with the `$(arg <argument name>)` syntax, as opposed to a property/param which uses the `${<property name>}` or `${<param name>}` syntax. You can see the argument being used later in the file:
```xml
<xacro:rrbot prefix="$(arg prefix)" parent="world" >
    <origin xyz="0 0 0" rpy="0 0 0" />          <!-- position robot in the world -->
</xacro:rrbot>
```

Another difference between Step 5 and Step 6 is the `$(find)` command. Because `rrbot_description` is now a sourced ROS package, we can use the `find` tool to find the absolute path of the `rrbot_description` package on our computer.
```xml
<xacro:include filename="$(find rrbot_description)/urdf/rrbot/rrbot_macro.xacro"/>
```
This feature is extremely useful when you are creating packages which utilize other packages. For example, if we wanted to mount a 3rd party robot arm onto the table we built in Step 5.

---

Moving on to the `rrbot/rrbot_macro.xacro` file, the main difference is the addition of the `prefix` parameter, which is simply inserted at the beginning of each joint/link name. The reason for a prefix parameter is so if there are multiple robots that share the same joint/link name, the launch file can add a prefix to one of the robots to resolve the name conflict. This is standard practice in industry settings, so we will keep it for the remainder of the course. The default value of prefix is simply an empty string "".

---

Finally, navigate to `src/rrbot_description/launch/view_rrbot.launch.py`. There are two launch arguments: the `description_package` and `prefix`. The `description_package` defaults to `rrbot_description`, which we won't need to change. The `prefix` is passed to the `xacro` tool via the code:
```python
robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "rrbot.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
```
which simply calls
```bash
xacro /path/to/rrbot.urdf.xacro prefix:=<prefix> 
```
where `<prefix>` is replaced with whatever argument that we pass in, of course. You can add a prefix to the rrbot by simply running the command:
```bash
ros2 launch rrbot_description view_rrbot.launch.py prefix:=my_awesome_
```
*Note: you may have to change the `Global Options > Fixed Frame` setting in RViz!*

## Task 1
Create a package called `table_description` in the same manner as Step 6 (robot name should be "table"). You should use `${prefix}` for each joint/link name. You are encouraged to reuse as much of your Step 5 code as you would like. The purpose of this exercise is really to learn how to create and use a proper description package. 

---

*Tip: make sure you run the setup tools in the correct directories! If you need to start over, you can **very carefully** delete your `table_description` package by navigating to the parent directory of `table_description` and running*:
```bash
rm -rf table_description
```
and then 
```bash
rosd
rm -rf build/ install/ log/
```
**Be extremely careful when running rm commands. Once you run them, the files will be gone forever, and there is no way to recover them unless they were commited to git and pushed to a remote repository already.** 

---

Known pitfalls:
* The brown color defined in `/urdf/common/materials.xacro` doesn't appear very brown. You can use a different color, or you can replace it with this color:
```xml
<material name="brown">
    <color rgba="0.54 0.27 0.07 1.0"/>
</material>
```
* The default `.rviz` settings use `base_link` as the fixed frame, which may not exist, causing the table to appear completely white with the legs in the wrong position. Simply change the `Global Options > Fixed Frame` to `world` and the problem will go away.
* If the RViz window is flickering, try resizing the window (drag the corner of the window in). Sometimes the ThinLinc window cannot display above a certain window size without tearing.
* You may wish to remove the `joint_state_publisher_gui` from the launch file since all of your table joints are fixed.
* If ROS can't find you package, try sourcing your local workspace by running `_ws1` or equivalently, `rosd && source install/setup.bash`.
---

## Task 2
Create another package called `rrbot_on_table_description` in the same manner as step 6 (robot name should be "rrbot_on_table"). You can delete the entire `urdf/rrbot_on_table` folder, as you will only need the `rrbot_on_table.urdf.xacro` file, which is included in the start code. Verify that you can move the robot arm on the table. Take a screenshot.

## Next Step
Proceed to [Step 7](/Step7)
