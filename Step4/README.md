# Step 4: Introduction to URDF
The next four tutorials will be focused on creating URDF (Universal Robot Description Format) files. URDF files are XML-based (Extensible Markup Language) text files which describe our robot. We can specify links and joints, and then use RViz (ROS visualizer) to move our links around. You can find the full URDF specification online [here](https://wiki.ros.org/urdf/XML).

In Step 4, you will learn how to create URDF files from scratch. In Step 5, we will use the `xacro` tool to stream-line the process. In Step 6, we will become familiar with creating a robot description as a ROS package via `setup-robot-description`. In Step 7, we will look at the official Universal Robotics Description Package, and get to visualize the arm moving! 

## Running the Example
We will create a simple robot named `rrbot`, which is a robotic arm with two revolute joints. Change into the `Step4` directory and then run the command:
```bash
ros2 launch rrbot.launch.py
```
and you will see RViz open and a GUI with two sliders appear. There are some settings we need to configure in RViz.
1. In the left panel, change `Fixed Frame` from `map` to `world`.
2. In the bottom left corner above the `Time` window, click the `Add Button`. A window will appear. Scroll down and select `RobotModel`, and then press `OK`.
3. In the left panel, expand `Robot Model`. Then set the `Description Topic` to `/robot_description`. The robot should appear, and moving the sliders in the GUI should move the robot arm joints.
4. Click `Add Button` and select `TF` and then press `OK`. Verify that moving the sliders moves the frames along with the robot.
5. Press `File > Save Config` to save your `rrbot.rviz` settings file. Next time you run the launch script, you won't need to repeat steps 1-4.

_Note: your RViz window might appear to be flashing. This is a graphical issue caused by the ThinLinc remote desktop client. Try making the RViz window smaller (drag the upper left corner of the RViz window towards the middle) and/or putting the ThinLinc client window onto a different monitor._

## URDF File Structure
Open the `rrbot.urdf` file. This file describes the visual properties of the rrbot. The file starts with a XML declaration and then a robot tag.
```xml
<?xml version="1.0"?>
<robot name="rrbot">
    <!-- all of the robot code goes here -->
</robot>
```
Since the `rrbot.urdf` file is our "main" URDF flie (it's our only URDF file), we need to add the `name` attribute to the `<robot>` tag. 

*Tip: you can view the `.urdf` files with text highlighting in VSCode by configuring the file association for `.urdf` files. Open a `.urdf` file. In the bottom right corner of VSCode, there is a blue banner. Click on `Plain Text`, then `Configure File Association for .urdf`, and then select `XML`.*

Next, we declare some materials:
```xml
<material name="black">
    <color rgba="0.0 0.0 0.0 1.0" />
</material>
<material name="red">
    <color rgba="0.8 0.0 0.0 1.0" />
</material>
<material name="green">
    <color rgba="0.0 0.8 0.0 1.0" />
</material>
```
The colors are specified with their red, green, blue, and alpha (transparency) colors, each represented as a float between 0 and 1, inclusive.

It is common practice to define a world frame which represents the origin of our coordinate system.
```xml
<link name="world" />
```

Each arm of the rrbot is represented with a `<link>` tag, and `<joint>` tags are used to join links together. Here is the `base_link` of the robot, which is a black box which measures `5cm x 5cm x 2m`. The center of the box is drawn 1 meter in the air (`xyz="0 0 1.0"`) so that the `base_link` frame appears at the bottom of the visual box.
```xml
<link name="base_link">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 1.0" />
        <geometry>
            <box size="0.05 0.05 2" />
        </geometry>
        <material name="black" />
    </visual>
</link>
```
_Note: as per the [official URDF documentation](https://wiki.ros.org/urdf/XML/link) there are only four types of geometry tags: box, sphere, cylinder, and mesh. Read the documentation for more info._


Links typically also have a `<collision>` and `<inertia>` tag, but we won't use them until Step 5.

Joints are used to attach links together and define their range of motion relative to each other. Because we want to bolt the `base_link` to the `world`, we used a `fixed` joint:
```xml
<joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0" />
    <parent link="world" />
    <child link="base_link" />
</joint>
```
The order of the links and joints in the URDF file does not matter. For clarity, the joints are placed between the parent and child `<link>` tags in the URDF file.

*Note: the `rpy` values stand for roll, pitch, and yaw, which describe the rotation of the child relative to the parent about different axes. For this tutorial, we will just have `rpy="0 0 0"`.*

Next we define Link 1, which is a red box measuring `5cm x 5cm x 1m`. 
```xml
<link name="link1">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0.45" />
        <geometry>
            <box size="0.05 0.05 1" />
        </geometry>
        <material name="red" />
    </visual>
</link>
```
If not specified, the visual origin defaults to the visual middle of the link. However, we specify the visual origin in order to shift it until it appears 5cm above the bottom of the joint, (hence why you see $0.45 = 1.0/2-0.05$ show up). All links must pivot a vector passing through their origin. Take a look at RViz again and verify that `link1` is rotating about its y-axis (green). Joint 1 connects the `base_link` and `link1` as a continuous joint. *Note: a continuous joint is just a revolute joint without upper and lower limits. You will explore revolute joints in Task 1.*
```xml
<joint name="joint1" type="continuous">
    <parent link="base_link" />
    <child link="link1" />
    <origin rpy="0 0 0" xyz="0 0.05 1.95" />
    <axis xyz="0 1 0" />
    <limit effort="1000" velocity="10.0" />
</joint>
```
Pay special attention to the `<origin>` and `<axis>` tags. Observe that the joint is placed at a height of 1.95 meters, (equal to the height of the base link minus the 5 cm offset). Play around by changing this value and seeing what happens.

Link 2 and joint 2 are defined similarly to link 1 and joint 1. At the end, we have a `tool0` frame. This is the location of the "end effector" (sometimes abreviated to EE) of the robot. A tool such as a gripper or drill could be attached to this point.
```xml
<joint name="tool_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.95" />
    <parent link="link2" />
    <child link="tool0" />
</joint>

<link name="tool0" />
```

## Launch Script
Now open `rrbot.launch.py`. We see three nodes are created:
* `/joint_state_publisher_gui`
* `/robot_state_publisher`
* `/rviz2`

You can verify this by running the launch script (always use `ros2 launch` to launch a launch script) and typing `ros2 node list`. The `/robot_state_publisher` node is responsible for taking in the URDF file and joint information and outputting the position and orientation of the robot frames. The `/joint_state_publisher_gui` provides a GUI (graphical user interface) with sliders to allow us to change the joint positions, which are sent over the `/joint_states` topic. In fact, we could publish our own message to the `/joint_states` topic to move to the robot ourselves. We will come back to this idea later.

_Note: you might see a fourth node starting with `transform_listener_impl_` that is created when rviz2 is opened. This is normal._

## Task 1
The goal of Task 1 is to create a new robot called r3bot (three revolute joints). 
* To get started, copy your `rrbot.launch.py` and `rrbot.urdf` and `rrbot.rviz` files into `r3bot.launch.py` and `r3bot.urdf` and `r3bot.rviz`.
* Modify your r3bot launch script to reference the new `r3bot` files. Launch the script to verify you can see the original rrbot.
* Visit [the URDF joint specification](https://wiki.ros.org/urdf/XML/joint) to learn about revolute joints. Change the `joint1` and `joint2` to type `revolute` with a range of `[-1.57, 1.57]` radians. *Hint: you will need to change the `type` to `revolute`, and then add `lower` and `upper` attributes to the `<limit>` tag.* Visualize the robot in Rviz to verify the limits are respected in the joint state publisher gui.
* Add a `link3` and `joint3` to your r3bot. Ensure `joint3` is a revolute joint with upper and lower limits of `+/- 1.57` radians. Change the color of link 3 to `blue`. Modify the `tool0_joint` so it is on the end of `link3`. (You just need to replace the `TODO` text with the proper values.)
```XML
<!-- Joint 3 -->
<joint name="joint3" type="TODO">
    <parent link="TODO" />
    <child link="TODO" />
    <origin rpy="0 0 0" xyz="0 0.05 0.9" />
    <axis xyz="0 1 0" />
    <limit effort="1000" velocity="10.0" lower="TODO" upper="TODO" />
</joint>

<!-- Link 3 -->
<link name="link3">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0.2" />
        <geometry>
        <box size="0.05 0.05 0.5" />
        </geometry>
        <material name="TODO" />
    </visual>
</link>

<!-- Joint from link3 to tool0 -->
<joint name="tool_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.45" />
    <parent link="TODO" />
    <child link="tool0" />
</joint>
```
* Take a screenshot of your robot with the TFs visible. Set all limits their maximum value.


## Task 2
Using your knowledge of the URDF format, create a 4-legged rectangular table in `table.urdf` with the following requirements:
 * Table Top (box): `2m length, 1m width, 5cm thickness`
 * Table Legs ([cylinder](https://wiki.ros.org/urdf/XML/link)): `5cm radius, 95cm length`
 * The table top should be placed on top of the legs so that the surface of the table top is `1m` above the ground.
 * The origin of the table top should be centered **on top of the table**. That is, when you look at RViz, you should see that the `table_top` link is sitting on top of the table. This will be important for Steps 5 and 6, where we begin to mount a robotic arm onto the top surface of the table. (We don't want to attach the robotic arm to the center of mass of the table top). Please see the figure below for an illustration of where the table_top frame should be placed.

![image](https://media.github.itap.purdue.edu/user/7114/files/20df0908-0c71-469d-bb9f-839ece5b6a37)

 * The origin of the legs should be centered **on the top of each leg**.
 * All joints should be `fixed` type.
 * The table legs should be completely under the table top.
 * Make the table top `brown`, and the legs `light_grey`. You will need to define your own material colors.
 ```xml
 <material name="brown">
     <color rgba="0.54 0.27 0.07 1.0"/>
 </material>
 <material name="light_grey">
     <color rgba="0.8 0.8 0.8 1.0"/>
 </material>
 ```
 * Create a launch script (`table.launch.py`) to view your table. You may omit the `/joint_state_publisher_gui` node since all of the joints are fixed.
 * *Hint: the legs should be children of the table top. That is, table top will have four children: leg1, leg2, leg3, and leg4.*
* Be sure to take a screenshot when you are complete.

## Next Steps
Proceed to [Step 5](/Step5)
