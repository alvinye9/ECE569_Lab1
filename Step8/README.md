# Step 8: Publishing to the Joint State Topic
Until now, we have used the `joint_state_publisher_gui` node to publish the joint data for our robot. Now, we will learn to create our own ROS node which publishes our own joint data.

Run the command
```bash
ros2 launch ur3e_on_table_description view_ur3e_on_table.launch.py
```
and then (in another terminal) run
```bash
ros2 topic list -t
```
and observe that `/joint_states` is of type `sensor_msgs/msg/JointState`. Run the command
```bash
ros2 interface show sensor_msgs/msg/JointState
```
to view how a `sensor_msgs/msg/JointState` message is structured. 
Now run
```bash
ros2 topic echo --once /joint_states
```
to view the actual `sensor_msgs/msg/JointState` message being sent from the joint state publisher gui. You should see something like this:
```yaml
header:
  stamp:
    sec: 1720633538
    nanosec: 252970928
  frame_id: ''
name:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
velocity: []
effort: []
---
```

## Task
Create a new ROS package called `py_joint_pub` where we can write Python code to publish `sensor_msgs/msg/JointState` messages to the `/joint_states` topic to move the robot. Create a file called `joint_publisher_test.py` and `joint_publisher_lissajous.py`, and populate them with starter code provided. See [Step 3](/Step3) for a refresher on creating a ROS package and a publisher node. Your `package.xml` file should contain the following dependencies (simply the import statements in our python files):
```xml
    <exec_depend>rclpy</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>
    <exec_depend>numpy</exec_depend>
    <exec_depend>pandas</exec_depend>
```
Also copy and paste the `lissajous.csv` file into the `resource` folder. Next, modify your `setup.py` file as follows:
```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/lissajous.csv'])
    ],
    # ...
    entry_points={
        'console_scripts': [
            'test = py_joint_pub.joint_publisher_test:main',
            'lissajous = py_joint_pub.joint_publisher_lissajous:main',
        ],
    },
```
Now, you can launch the `ur3e_on_table_description` package, and then run the test code:
```bash
ros2 run py_joint_pub lissajous
```
You should see the robot moving. Now, in RViz select `RobotModel > Links > tool0 > Show Trail`. You should see a blue lissajous curve (figure of 8) appear. Take a screenshot.

---

Stop the `joint_publisher_lissajous` node with `Ctrl+C`. Now, you can go ahead and run:
```bash
ros2 run py_joint_pub test
```
and see the robot moving. This example uses time to calculate the robot joint position:
```python
msg.position = [2*np.pi*t/5, -1+0.5*np.cos(2*np.pi*t/10), np.sin(2*np.pi*t/15), 0.0, 0.0, 0.0]
```

Use the `joint_publisher_test.py` file as a starting point for the bonus task:

## Bonus Task: Linear Interpolation

Congrautulations on getting this far in Lab 1! You have learned a lot about ROS, URDF, and XACRO files. Now let's combine these all together, while adding some primative motion planning capabilities for some extra credit ☺️

Your task is to create a new executable called `pick-and-place` within the `py_joint_pub` package which can move between two positions in space. We will define two joint position vectors $p_1$ and $p_2$, which represent the beginning and end of the trajectory. These vectors live in $\mathbb{R}^6$ because the robot arm has 6 joints. We can interpolate between the two joint positions linearly with a parameter $0 \le a \le 1$ as follows:

$$
p = (1 - a) p_1 + a p_2
$$

where $p$ represents the current joint position. Note that when $a=0$, $p=p_1$; when $a=1$, $p=p_2$. 

We want to start at $p_1$, wait 5 seconds, move to $p_2$, wait 5 seconds, and then move back to $p_1$, before repeating the process. Here is some starter code which implements this behavior:

```python
# other imports...
import numpy as np

def linear_interpolate(p1, p2, a):
    return (1-a)*p1 + a*p2

class JointPublisherPickAndPlace(Node):

    def __init__(self):
        # other code ...

        # define your start/end points
        self.p1 = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.p2 = np.array([-1.0,-1.0,-1.0,-1.0,-1.0,-1.0])

    def timer_callback(self):

        # 0..5 seconds: stay at p1
        # 5..10 seconds: move towards p2
        # 10..15 seconds: stay at p2
        # 15..20 seconds:: move towards p1
        if self.t < 5:
            a = 0
        elif self.t < 10:
            a = (self.t - 5)/5
        elif self.t < 15:
            a = 1
        elif self.t < 20:
            a = 1 - (self.t - 15)/5
        
        p = linear_interpolate(self.p1, self.p2, a).tolist()

        # other code ...
        msg.position = p
        # other code ...

        # reset self.t after 20 seconds are up
        self.t += self.timer_period
        if self.t >= 20:
            self.t = 0
        
```

Combine this starter code with the `joint_publisher_test.py` code in order to create a new executable called `pick-and-place`. Visualize the robot moving in RViz with the trail enabled for the `tool0` link. Answer the following questions:
* Does the `tool0` joint move linearly through Cartesian space? Any ideas as to why or why not?
* Describe a situation for where linear interpolation through joint space would be good idea for motion planning.
* Describe a situation for where linear interpolation through joint space would be a poor idea for motion planning.




