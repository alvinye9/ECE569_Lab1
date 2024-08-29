# Step 2: Introduction to ROS Nodes, Topics
In this tutorial, you will learn how to 
* Run ROS nodes
* Publish and subscribe to messages from the terminal
* Analyze ROS message types
* Remap topics, nodes, and use namespaces
* Use launch files to run ROS nodes

## Before you start
Step 2 is based off of the following official ROS tutorials. You may wish to read them if you haven't used ROS before.
* [Understanding Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
* [Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

You may also want to bookmark the [ROS2 Humble Home Page](https://docs.ros.org/en/humble/index.html) for future reference.

## Running ROS Nodes
ROS comes with many example packages (a package is just a collection of nodes, written in C++ or Python). In this task, you will learn how to create nodes and publish messages to topics.

Open three terminal windows. In the first terminal, start the `talker` node by running:
```bash
ros2 run demo_nodes_cpp talker
```
You should the following output:
```
[INFO] [1718298748.329062227] [talker]: Publishing: 'Hello World: 1'
[INFO] [1718298749.329037334] [talker]: Publishing: 'Hello World: 2'
[INFO] [1718298750.329030574] [talker]: Publishing: 'Hello World: 3'
```

Go to the second terminal and run the `ros2 node list` and `ros2 topic list` commands. You should see the `/talker` node and the `/chatter` topic appear have appeared. (If you don't see any topics, ensure that messages are still printing in the first terminal. Pressing `Ctrl+C` will kill the `/talker` node.)

In the third terminal, run:
```bash
ros2 run demo_nodes_py listener
```
You should see something like:
```
[INFO] [1718299318.831494955] [listener]: I heard: [Hello World: 177]
[INFO] [1718299319.823964625] [listener]: I heard: [Hello World: 178]
[INFO] [1718299320.823881544] [listener]: I heard: [Hello World: 179]
```
Go back to the second terminal and run the `ros2 node list` and `ros2 topic list` commands again. You should see the `/talker` and `/listener` nodes and the `/chatter` topic present. 

Congratulations! You have successfully used ros2 to communicate between two separate programs written in two different programming languages!

Now stop the ros2 nodes by going to terminal 1 and terminal 3 and pressing `Ctrl+C`. Verify (from any terminal) that the nodes aren't active anymore using `ros2 node list`.

### Explanation
When you ran `ros2 run demo_nodes_cpp talker`, a node was created that published to the `/chatter` topic. Then, `ros2 run demo_nodes_py listener` created a node which subscribed to messages on the `/chatter` topic. When the listener receives a topic, a section of code is executed, which prints the contents of the `/chatter` topic to the screen.

## Publishing to a Topic
You can manually publish to a topic using the command line. This requires us to know the topic and message type. Start the `listener` node like before.
In another terminal, run the command 
```bash
ros2 topic list -t
``` 
to view the topic message types. We see that `/chatter` has type `std_msgs/msg/String`. Now run the command
```bash
ros2 interface show std_msgs/msg/String
```
to see the format of the message type (a single element `data` of type `string`).


Now, run:
```bash
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'my custom text'"
```
You should verify that the listener printed something like
```
[INFO] [1718302233.047970482] [foo.listener]: I heard: [my custom text]
```
Kill the listener node and continue.

## Listening to a Topic
You can also subscribe to a topic from the command line to see what messages are being published onto a topic. Start up a talker node and then run:
```bash
ros2 topic echo /chatter
```
You should see the individual messages separated by lines:
```
data: 'Hello World: 24'
---
data: 'Hello World: 25'
---
data: 'Hello World: 26'
---
```

## Remapping Nodes and Topics
It is also possible to run multiple instances of the same node. Two nodes cannot have the same name, so we need to remap them to a different name at run time. There are three different types of remapping:
* Changing a topic name via `--ros-args -r <your old topic name>:=<your new topic name>`
* Changing a node name via `--ros-args -r __node:=<your new node name>`
* Changing the namespace via `--ros-args -r __ns:=/<your new namespace>`

_Note: these are not commands on their own! They are to be added to a `ros2 run` command in order to change its functionality. See Example 1 below!_

Remapping the namespace will prefix the namespace to the front of the node name and its child topics' names (unless the child topics are meant to be "global" topics, such as `/tf` as you will encounter later). We will not be using this feature in the class, but it does exist.

#### Example 1
For example, the following commands (each in separate terminals) create two different talkers and listeners, talking on different topics.
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp talker --ros-args -r __node:=talker2 -r chatter:=chatter2
ros2 run demo_nodes_cpp listener
ros2 run demo_nodes_cpp listener --ros-args -r __node:=listener2 -r chatter:=chatter2
```

#### Example 2
The following commands (each in separate terminals) create two talkers, each publishing to the same topic.
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp talker --ros-args -r __node:=talker2
ros2 run demo_nodes_cpp listener
```
Running the command:
```bash
ros2 topic info /chatter
```
will output the message type, number of publishers, and number of subscribers of that particular topic. Give it a try!

_Aside: the astute reader may wonder why there is so much inconsistency in notation between remapping an topic vs a node vs a namespace. For example, with a topic, why do you need to specify the old and the new topic, but for a node you just specify the new name? Well, when you use `ros2 run` to run a node, you only create a single node, so ROS already knows what node you are renaming. But a node might have 50 different topics it publishers or subscribes to, so you have to be specific. You may also wonder why the `-r` is necessary? That just tells you that you are remapping. As a whole, ROS is known to have a few quirks, but you will get used to them with practice. As a researcher, I don't try to memorize all of the different ROS commands: I just remember that there is a way to change a topic or node name, and then check online what command I need to run. My favorite way of changing topic and node names is using a launch file, which you will learn next._

## Using Launch Files
As you may have realized, starting and stopping nodes requires a lot of terminals, and remembering to remap topics, nodes, and namespaces is tedious. We can use launch files to start our ROS commands easier. [There is a lot you can do with launch files](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/) but we will keep it simple for now.

Open up the `chatterbot.launch.py` file and explore a little bit. Then, navigate to `Lab1/Step2` (via the command `cd Lab1/Step2`) and launch the file with
```bash
ros2 launch chatterbot.launch.py
```
*Pro tip: type `ros2 launch cha` and then press `Tab` to save yourself some typing.*

You can still open a new terminal to verify the nodes and topics have been created. Then you can kill the ROS nodes by pressing `Ctrl+C`.

### Passing Arguments to Launch Files
We can also pass optional arguments to launch files in order to change their behavior. Run:
```bash
ros2 launch many_chatterbots.launch.py --show-args
```
to see a list of arguments that you can pass in. They are passed using the format `<arg>:=<value>`. For example:
```bash
ros2 launch many_chatterbots.launch.py num_pairs:=3
```
will create three pairs of talker and listener nodes. Remember to kill all of your ROS nodes with `Ctrl+C` when you are done with them!

## Visualizing ROS Nodes and Topics
The `rqt_graph` tool is useful for visualizing all of the nodes and topics active in ROS. To use it, simply run the command:
```bash
rqt_graph
```
in a terminal window. In the top left corner is a blue refresh button you may need to press. Try running the `many_chatterbots` example with `rqt_graph` to visualize all of the different nodes and topics. 

## Next Steps
Proceed to [Step 3](/Step3).
