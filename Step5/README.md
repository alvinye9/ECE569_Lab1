# Step 5: Using XACRO Files to Clean up URDF Files

From the previous tutorial, you learned how to manually create URDF files. However, there are a few major disadvantages to creating URDF files by hand:
* URDF files can get very long
* URDF files involve a lot of copy pasting
* Constants and parameters are not permitted
* Lacks mathematical operations
* Very difficult and tedius to make design modifications

The `xacro` tool (pronounced "zacro", stands for XML Macro) was created to help with these shortcomings. We will learn how to use `.xacro` files to create our rrbot and table again. There are two very good resources for learning `xacro` on [github](https://github.com/ros/xacro/wiki) or on the [offical ros wiki](https://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File). There are many features of `.xacro` files that we will not go into, such as adding conditional logic, loops, and embedding Python code, but you are always welcome to learn more with the links above!

## Running an Example
From the `Step5` folder, run:
```bash
ros2 launch rrbot.launch.py
```
An RViz window and `joint_state_publisher_gui` should appear, and you should be able to view the robot as normal. So what is the advantage of using `.xacro`? Well, the physical dimensions of the robot have been completely parameterized using constants and mathematical expressions. Go into the `rrbot_macro.xacro` file. You will see many properties:
```xml
<xacro:property name="mass" value="10" />
<xacro:property name="width" value="0.05" />
<xacro:property name="height1" value="2" />
<xacro:property name="height2" value="1" />
<xacro:property name="height3" value="1" />
<xacro:property name="axel_offset" value="0.05" /> 
```
feel free to play around with these values, and then run the launch script again. Just like magic, the robot has changed! This is one reason why URDF files are almost never written by hand - `xacro` just makes life easy!

## XACRO Summary
Here is a summary of what you can do with `.xacro`:

1. Define constants with `<xacro:property>`
2. Use mathematical operations such as `${width/2}` or `${length*cos(pi/3)}`
3. Include other files with `<xacro:include>`
4. Create custom macros which generates `.urdf` code for us!

For example, the following macro can be used to calculate the inertia matrix for a box, which is necessary for simulating the robot. As you will learn later in this course, the inertial matrix for a constant-density box, taken about the center of mass is given by

$$
J = \frac{m}{12}
\begin{pmatrix}
y^2+z^2 & 0 & 0 & \\
0 & x^2 + z^2 & 0 \\
0 & 0 & x^2 + y^2
\end{pmatrix}
$$

We can create a macro which automatically calculates this matrix for us, and properly formats our URDF code with the `<inertial>` tag. We define the following macro:
```xml
<xacro:macro name="box_inertial" params="x y z mass *origin">
    <inertial>
        <mass value="${mass}" />
        <xacro:insert_block name="origin" />
        <inertia ixx="${1.0/12 * mass * (y*y + z*z)}" ixy="0.0" ixz="0.0"
            iyy="${1.0/12 * mass * (x*x + z*z)}" iyz="0.0"
            izz="${1.0/12 * mass * (x*x + y*y)}" />
    </inertial>
  </xacro:macro>
```
Then, we call the macro as follows:
```xml
<xacro:box_inertial x="${width}" y="${width}" z="${height1}" mass="${mass}">
    <origin xyz="0 0 ${height1/2}" rpy="0 0 0" />
</xacro:box_inertial>
```
and this produces the following URDF code:
```xml
<inertial>
    <mass value="10"/>
    <origin rpy="0 0 0" xyz="0 0 1.0"/>
    <inertia ixx="3.3354166666666667" ixy="0.0" ixz="0.0" iyy="3.3354166666666667" iyz="0.0" izz="0.0041666666666666675"/>
</inertial>
```
Carefully look at what is going on here. We define the macro with `<xacro:macro name="box_inertial" params="x y z mass *origin">`. We see that x, y, z, mass, and origin are passed in as parameters. Based on this definition, we call the macro with `<xacro:box_inertial>`, and include the requried parameters. Notice that the origin is a special parameter because it starts with `*`: that means the origin parameter expects not a scalar value, but a `<origin>` tag.

**The rest of this tutorial will heavily rely on macros, so make sure you understand their syntax before continuing.** Macros are not limited to just `<inertial>` tags. Any valid `<xml>` code can make use of macros. For example, you could write a macro to generate a `<link>` or a `<joint>` tag.

## rrbot XACRO Walkthrough
First, open the `rrbot.urdf.xacro` file. This is our "top-level" file, which is called by the `xacro` tool to generate a URDF file. By convension, the top-level file will end with `.urdf.xacro`.
```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="rrbot">

  <xacro:include filename="rrbot_macro.xacro"/>

  <link name="world" />

  <xacro:rrbot parent="world">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:rrbot>

</robot>
```
First, we see that the `<robot>` tag has the attribute `xmlns:xacro="http://www.ros.org/wiki/xacro"`. This is required at the start of every `.xacro` file. 
The `<xacro:include>` tags copy and paste the contents of `rrbot_macro.xacro`,  `inertials.xacro`, and `materials.xacro`. 
Then the `world` link is defined.
Finally, the `<xacro:rrbot>` macro is called, which creates the rrbot with the specified `parent` parameter and specified `origin`.

The `<xacro:rrbot>` macro is defined in `rrbot_macro.xacro`, which you should open now.

---



First, we see one very large macro called `rrbot`, which makes up the majority of the file. The macro starts by defining some properties for the dimensions of the rrbot. Next, the `base_joint` is defined:
```xml
<joint name="base_joint" type="fixed">
    <xacro:insert_block name="origin" />
    <parent link="${parent}" />
    <child link="base_link" />
</joint>
```
We see that the `<origin>` block from the `rrbot.urdf.xacro` file is copied into this joint. Also, the parent is set to the value of the parameter `${parent}`, which in this case is `world`.

The base link includes mathematical expressions, as well as a call to the `box_inertial` macro:
```xml
<link name="base_link">
    <collision>
        <origin xyz="0 0 ${height1/2}" rpy="0 0 0" />
        <geometry>
            <box size="${width} ${width} ${height1}" />
        </geometry>
    </collision>

    <visual>
        <origin xyz="0 0 ${height1/2}" rpy="0 0 0" />
        <geometry>
            <box size="${width} ${width} ${height1}" />
        </geometry>
        <material name="blue" />
    </visual>

    <xacro:box_inertial x="${width}" y="${width}" z="${height1}" mass="${mass}">
        <origin xyz="0 0 ${height1/2}" rpy="0 0 0" />
    </xacro:box_inertial>
</link>
```
The `<collision>` tag has the same geometry as the `<visual>` tag. For more complicated robots, the collision tag will sometimes be a simplied version of the `<visual>` geometry. The `<xacro:box_inertial>` macro uses the `<xacro:property>`s which we defined above.

Take a look at the `<joint1>` tag:
```xml
<joint name="joint1" type="continuous">
    <parent link="base_link" />
    <child link="link1" />
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit effort="1000" velocity="10.0" />
</joint>
```
This is a good example of using mathematical operations for defining the location of the child link. The rest of the macro follows in a similar manner.

## Launch File
The `xacro` tool can be used from the command line to generate a URDF file. For example,
```bash
xacro rrbot.urdf.xacro > rrbot.urdf
```
will create a pure URDF file which we can run with a launch script. However, we can skip this step by having our launch file create the URDF file *in memory* without actually writing to a new file. Take a look at `rrbot.launch.py`:
```python
robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution("rrbot.urdf.xacro"),
    ]
)

robot_description = {"robot_description": robot_description_content}
```
This code is responsible for calling `xacro rrbot.urdf.xacro` and then passing the text to `robot_description`, which is then used when running the `robot_state_publisher` node.

## Tasks
1. Using what you have learned about `xacro`, recreate your four-legged rectangular table.
   * You should create only two files: `table.urdf.xacro` and `table_macro.xacro`.
   * You should create the following xacro properties:
     * `table_length = 2`
     * `table_width = 1`
     * `table_height = 1`
     * `table_thickness = 0.05`
     * `wood_density = 800`
     * `table_mass = ${table_length * table_width * table_thickness * wood_density}`
     * `leg_length = ${table_height - table_thickness}`
     * `leg_radius = 0.05`
     * `aluminum_density = 2700`
     * `leg_mass = ${pi*leg_radius*leg_radius*leg_length*aluminum_density}`
   * Since all four legs are identical, you should reduce code duplication by creating a macro named `leg` with the parameter `name`, which generates a `<link>` tag for you:
  ```xml
<xacro:macro name="leg" params="name">
    <link name="${name}>">
        <!-- more code here ... -->
    </link>
</xacro:macro>
  ```
  * Call this macro four times:
  ```xml
<!-- create four legs -->
<xacro:leg name="leg1" />
<xacro:leg name="leg2" />
<xacro:leg name="leg3" />
<xacro:leg name="leg4" />
  ```
   * All links should have the proper visual, collision, and inertial properties.
     * The table top is solid oak wood (density is $800 kg/m^3$) and the legs are solid aluminum (density is $2700 kg/m^3$). Use a `brown` color for the table top and a `light_grey` color for the legs.
  
2. Mount the robot on the table.
   * Create a launch script called `robot_on_table.launch.py` that visualizes the robot on the the table. You will need a `robot_on_table.urdf.xacro` file like this:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="table">

    <xacro:include filename="inertials.xacro" />
    <xacro:include filename="materials.xacro" />
    <xacro:include filename="table_macro.xacro" />
    <xacro:include filename="rrbot_macro.xacro" />

    <link name="world" />

    <xacro:table parent="world">
        <origin xyz="0 0 1" rpy="0 0 0" />
    </xacro:table>

    <xacro:rrbot parent="table_top">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:rrbot>

</robot>
```
   * *Hint: ensure that the joints/links between the robot and table are all unique, otherwise the robot won't show up in RViz*
3. In a similar fashion, create a `table_on_robot.urdf.xacro` file which mounts the child `table_top` frame to parent `tool0`.
```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="table">

    <xacro:include filename="inertials.xacro" />
    <xacro:include filename="materials.xacro" />
    <xacro:include filename="table_macro.xacro" />
    <xacro:include filename="rrbot_macro.xacro" />

    <link name="world" />

    <xacro:rrbot parent="world">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:rrbot>
    
    <xacro:table parent="tool0">
        <origin xyz="0 0 0" rpy="${pi} 0 0" />
    </xacro:table>

</robot>
```

## Next Steps
Proceed to [Step 6](/Step6)