# Step 7: Universal Robotics ROS2 Description Package
In this tutorial, you will download the official Universal Robotics ROS2 Description Package and move the ur3e robot around in RViz! This is the same robot that we have in MSEE B22, which you can visit during in-person office hours.

## Running the Example
Since this ROS package has been installed globally for all users on eceprog already (in `/opt/ros/humble/lib/`) and the `/opt/ros/humble/setup.bash` has already been sourced, the description package can be run by simply running:
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur3e
```
Move the sliders around to move the robot.

## Source Code on Github
You can find the Universal_Robots_ROS2_Description repository [on Github](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble). Make sure you have the "humble" branch selected.

Go into the `urdf` folder, and take a look at the `ur.urdf.xacro`. First, we see a `<xacro:include>` which copies the `ur_macro.xacro` code.

Next, we see many `<xacro:arg>` defined, which are passed from the launch script in `launch/view_ur.launch.py`. The most important argument is the `ur_type`, which we will set to `ur3e` to match the robotic arm in lab. Another argument of interest is as follows:
```xml
<xacro:arg name="joint_limit_params" default="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"/>
```
Notice how the `joint_limit_params` file path is found using the `$(find)` command to navigate to the proper configuration `.yaml` file, which is located at `config/ur3e/joint_limits.yaml`. In the `ur_common.xacro` file, the `.yaml` is read using the `xacro.load_yaml()` command, and then subsequently parsed into parameters.
```xml
<xacro:property name="config_joint_limit_parameters" value="${xacro.load_yaml(joint_limits_parameters_file)}"/>
<xacro:property name="sec_limits" value="${config_joint_limit_parameters['joint_limits']}"/>
<xacro:property name="shoulder_pan_lower_limit" value="${sec_limits['shoulder_pan_joint']['min_position']}" scope="parent"/>
```
Putting parameters into `.yaml` files is nice because they are easier to read/edit than hardcoded `.xacro` properties. This technique is something for you to consider when you make more URDF files in the future.


After this, the `<world>` link is defined and the `xacro:ur_robot` macro is called. Towards the end of the file are two `<xacro:if>` statements which are used for simulation purposes (either Gazebo Classic or Ignition Gazebo). We will not be using Gazebo until Lab 4, so don't worry about this code.

---

Next, open `ur_macro.xacro`. First, we see two `<xacro:include>` statements are defined. The `ur_common.xacro` file contains a few macros and many properties. The `ur_transmissions.xacro` contains `<transmission>` tags which are beyond the scope of this course. We won't be needing them in the class. 

Then, we see many links and joints, similar to what we have seen with `rrbot`. Let's take a look at one `<link>` definition:
```xml
 <link name="${tf_prefix}forearm_link">
    <visual>
        <origin xyz="0 0 ${elbow_offset}" rpy="${pi/2} 0 ${-pi/2}"/>
        <geometry>
            <xacro:get_mesh name="forearm" type="visual"/>
        </geometry>
    </visual>
    <collision>
        <origin xyz="0 0 ${elbow_offset}" rpy="${pi/2} 0 ${-pi/2}"/>
        <geometry>
            <xacro:get_mesh name="forearm" type="collision"/>
        </geometry>
    </collision>
    <xacro:cylinder_inertial radius="${forearm_inertia_radius}" length="${forearm_inertia_length}"  mass="${forearm_mass}">
        <origin xyz="${-0.5 * forearm_inertia_length} 0.0 ${elbow_offset}" rpy="0 ${pi/2} 0" />
    </xacro:cylinder_inertial>
</link>
```
There are two macros called within this `<link>` definition, all of which live in `ur_common.xacro`. The `xacro:get_mesh` macro sets up the `<geometry>` tag to accept meshes, which are created in CAD files. This is why the robot arm is so detailed in RViz. The `xacro:cylinder_inertial` is similar to what we have seen in Step 5 and 6.

---

While this description package is more complicated than our `rrbot` package, there isn't that many new concepts. If you've understood up to this point farily well, you are more than equipped to create description packages for any robot you create in the future!

## Task: Putting the UR3e Arm on the Table
Create a new description ROS package as shown in Step 6, with the name `ur3e_on_table_description`. Then, delete all of the existing files in the `/urdf` folder, and add the `ur3e_on_table.urdf.xacro` file from the Step 7 starter code to the `/urdf` folder. (Note: you don't have to write any code for this step - simply move the .xacro file I wrote to the correct location.) Please read through this file so you get a sense of how you would go about attaching the robotic arm to something else (the .xacro file should look a lot like a combination of `ur.urdf.xacro` from Github and `rrbot_on_table.urdf.xacro` from Step 6). Then simply run:
```bash
ros2 launch ur3e_on_table_description view_ur3e_on_table.launch.py
```
* You should be able to view and move the ur3e arm on the table. Note that the $-135$ degree yaw is so that the robot is in the same orientation as the one in lab.
* Tip: make sure that `world` is selected as the fixed frame in RViz, *not* `base_link`. Otherwise it will appear the table is rotated shifted to that the top aligns with the grid.

## Next Steps
Proceed to [Step 8](/Step8)
