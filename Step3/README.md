# Step 3: Introduction to ROS Packages (Python)
In this tutorial, you will learn how to
* Create a ROS workspace
* Create a ROS package
* Write a ROS node in Python to publish to a topic
* Write a ROS node in Python to listen to a topic

This tutorial is based on the official ros tutorial [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

## ROS Workspace
All of our ROS packages need to live in a local ROS workspace. We will create a `ws1` folder and then make it into a workspace to house all of our packages for Lab 1. Now, do the following:
```bash
cd ~/ece569-fall2024/Lab1
mkdir ws1
setup-ros-workspace ws1 humble
```
Press `ENTER` to the prompts. After setup is complete, close your terminal, then open a new terminal and run the command
```bash
_ws1
```
to source the new workspace and then run:
```bash
rosd
```
to enter the new workspace. Now the workspace is all set up! You should see something like `<your username>@eceprog4:[ws1]<main>ws1$` in your terminal, indicating that you have sourced the `ws1` workspace and are on the `<main>` branch on your Github repo. If you don't, open a new terminal and run `_ws1`.

*Tip: now that the `ws1` workspace has been sourced with `_ws1`, there are are a few aliases available on the command line [here](https://rtw.stoglrobotics.de/master/use-cases/ros_workspaces/aliases.html). For example, you can change into the `src` folder of your workspace by running `rosds`.*

## ROS Package
We will create a package with two nodes: a talker (publisher) and a listener (subscriber). We will call this package `py_pubsub`. We will use the `create-new-package` command from [here](https://rtw.stoglrobotics.de/master/tutorials/quick-start.html).

First, `cd` into the `~/ece569-fall2024/Lab1/ws1/src` folder, then create a new package:
```bash
create-new-package py_pubsub "a simple publisher and subscriber in python"
```
You will need to follow the prompts as follows:

```
(1) standard
(2) global git: <your name and email>
(4) Apache-2.0
(2) ament_python
(no) setup/update repository with new package configuration
```
_Note: do not include the parenthesis `( )` in your prompts. Just type the number or string directly._

Verify the package was created successfully by running the command:
```bash
tree
```
from the `src` folder and seeing the following file structure:
```
.
└── py_pubsub
    ├── LICENSE
    ├── package.xml
    ├── py_pubsub
    │   └── __init__.py
    ├── resource
    │   └── py_pubsub
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py

4 directories, 9 files
```
Finally, build the package with
```bash
rosd && colcon build --symlink-install --packages-select py_pubsub
```
If you see an `EasyInstallDeprecationWarning` you can ignore this error.

## Create Publisher
Create a new file in `src/py_pubsub/py_pubsub/` named `publisher.py` (next to the `__init__.py` file). Copy and paste its contents from `Lab1/Step3/publisher.py` file. Now we must add some dependecies:

1) In the `package.xml` file, add the following lines between the `</license>` and `<test_depend>` tags:
```xml
    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
```
*These python modules are imported in the `publisher.py` script, which is why they are needed here.*

2) In the `setup.py` file, modify the `entry_points` as follows:
```python
    entry_points={
        'console_scripts': [
            'publisher = py_pubsub.publisher:main',
        ],
    },
```
and be sure the indentation levels are consistent. This allows us to call the `main()` function in the `publisher.py` file when we run `ros2 run py_pubsub publisher` later.

3) Source the workspace by running: 
```bash
_ws1
```
_Note: You need to source the workspace for `rosd` to work in the next command._

4) Build the package with
```bash
rosd && colcon build --symlink-install --packages-select py_pubsub
```

5) Source the terminal again after you build.
```bash
_ws1
```
_Note: sourcing your workspace is a common occurence when you are creating ros packages. If ros can't find your package or launch file, chances are you need to just source your workspace again._


### Operation
You can run your publisher using the command
```bash
ros2 run py_pubsub publisher
```
You should check the node name and topic name using `ros2 node list` and `ros2 topic list`.

### Tasks
1) Modify the `publisher` node so its node name is `my_publisher` (instead of `minimal_publisher`) and publishes to the topic `chatter` (instead of `my_topic`). Also replace the string `Hello World` with something more creative. Start your publisher node, and verify that 
```bash
ros2 run demo_nodes_cpp listener
```
can hear your messages. 
_The point of this exercise is to get you familiar with editing Python code using VSCode. While you can use the command line to change the node and topic names, I want you to get practice changing Python code in order to change the functionality of your ROS package._

*Tip: because you didn't create any new files and we already built the package with the `--symlink-install` flag, you don't actually need to rebuild the package for this task. The `--symlink-install` flag is handy for making modifications to existing files without the need to rebuild the entire package. This feature is especially when working with very large packages.*

## Create a Listener
Create a new file called `subscriber.py` in the same directory as `publisher.py`. Copy and paste the starter code. Because the import statements don't depend on any additional modules, the `package.xml` file can be left alone. However, we will have to add a new entry point in `setup.py`:
```python
    entry_points={
        'console_scripts': [
            'publisher = py_pubsub.publisher:main',
            'subscriber = py_pubsub.subscriber:main',
        ],
    },
```
Then, build the package with:
```bash
rosd && colcon build --symlink-install --packages-select py_pubsub
```
*(This is necessary because we created new files in our package.)*

### Tasks
2) Modify the subscriber node name to `my_subscriber` and the topic to `chatter`. Change the `I heard: ` message to something more creative. Run your `publisher` and `subscriber` nodes at the same time, and verify they can here each other! This task should also be done by modifying Python code.

## Next Steps
Proceed to [Step 4](/Step4).
