# Sensor Integration

In this session, we work with a simulated mobile robot to integrate
sensors such as lidar and an Inertial Measurement Unit (IMU). At the end of this session you'll have a simulated mobile robot driving around.

Using 3D simulators such as [Gazebo](https://gazebosim.org), we can attach simulated sensors to simulated robots to visualise their sensor data and evaluate different robot algorithms and behaviours.

## Preparation
To complete this session you will need to have the following packages installed:
```bash
sudo apt install liburdfdom-tools
sudo apt install ros-$ROS_DISTRO-tf2-tools
sudo apt install ros-$ROS_DISTRO-husky-simulator
sudo apt install ros-$ROS_DISTRO-husky-viz
```

<details>
<summary>Notes on using <code>$ROS_DISTRO</code></summary>

* Using the `$ROS_DISTRO` environment variable will ensure that you install the correct application for your ROS installation
  * i.e. Running the command: `echo $ROS_DISTRO` should return `noetic` for a `noetic` installation
* Note: If nothing is returned, you may not have "sourced" your ROS environment correctly
  * i.e. For a correctly sourced installation, running the command: `printenv | grep ROS` should return a list of `ROS` prefixed environment variables
</details>


## URDF Files

The [Unified Robot Description Format (URDF)][urdf] is an XML format that describes a robot's hardware, including it's chassis, linkages, joints, sensor placement, etc. Take a look at NASA's [Robonaut](https://github.com/gkjohnson/nasa-urdf-robots) to see what's possible with robot URDFs.

In the first part of this session, we'll add a simulated arm and gripper to  a mobile robot-- we'll then work up to controlling its movement in simulation.

### Create the robot model

In this section, we'll work with the
[robot_description package](./src/robot_description) in this repository.
Please go through the URDF file `robot.urdf` in
[its urdf directory](./src/robot_description/urdf) which describes our robot.

Check whether the model is complete, and the list of components using the below
commands.

```bash
cd src/robot_description/urdf
check_urdf robot.urdf
```

The following output will be shown.

```bash
robot name is: robot1
---------- Successfully Parsed XML ---------------
root Link: base_link has 3 child(ren)
    child(1):  wheel_1
    child(2):  wheel_2
    child(3):  wheel_3
```

#### Launch the robot simulation

To launch a simulation of an URDF modelled-robot, we will need to create a `launch` file

* Create a ROS-launch file named `display.launch` in [`src/robot_description/launch`](src/robot_description/launch). 
* Populate it with the following content
  ```XML
  <?xml version="1.0"?>
      <launch>
          <arg name="model" />
          <arg name="gui" default="False" />
  
          <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />
          <param name="use_gui" value="$(arg gui)"/>
          
          <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
          <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
          <node name="rviz" pkg="rviz" type="rviz" args="-d $(find urdf_tutorial)/urdf.rviz" />
      </launch>
  ```
    **Note**: The following packages are used in this `launch` file and were installed when you installed the `desktop` or `desktop-full` version of ROS.
    * **joint_state_publisher**: https://wiki.ros.org/joint_state_publisher
    * **robot_state_publisher**: https://wiki.ros.org/robot_state_publisher
    * **urdf_tutorial**: https://wiki.ros.org/urdf_tutorial
       
We'll come back to the nodes [`joint_state_publisher`][ros-joint-state-publisher] and [`robot_state_publisher`][ros-robot-state-publisher] soon. 
For now, let's launch the simulation by doing the following:

* Build the `robot_description` package to make it a ROS-package. From the `sensor-integration` directory run:
  ```bash
  catkin build robot_description
  ```
* Source the development package path, to ensure `robot_description` package is discoverable in your shell environment (e.g. callable while using `rospack` command).
  ```bash
  source devel/setup.bash
  ```
* Launch the modelled-robot through the `display.launch` and `robot.urdf` files that we just created.
  ```bash
  roslaunch robot_description display.launch model:=`rospack find robot_description`/urdf/robot.urdf use_gui:=true
  ```
* An RViz window should launch. Make the following changes:
  * Set "Fixed Frame" to `base_link`. (**Note**: RViz sets this to `map` by default)
  * Add `RobotModel` display via the "Add" button.

If successful you should see an output similar to the following image

![rviz_window](./resources/images/rviz_launch.png)

##### Brief description of URDF file

We break down the parts of `robot.urdf` briefly in this section.

* [`<link>`][urdf-link] element: Describes a rigid body with an inertia, visual features, and collision properties.
  * In the below snippet, we describe the base of the robot `base_link` which has a box-geometry with visual attributes like its color.

  ```XML
  <link name="base_link">
      <visual>
          <geometry>
              <box size="0.2 .3 .1"/>
          </geometry>
          <origin rpy="0 0 0" xyz="0 0 0.05"/>
          <material name="white">
              <color rgba="1 1 1 1"/>
          </material>
      </visual>
  </link>
  ```

* [`<joint>`][urdf-joint] element: Describes the kinematics and dynamics of a joint between two _links_, along with its safety limits. 
  * In the below snippet, the _joint_ description provides details on how the *base_link* and *wheel_1* links are connected. 
  * Take note of the joint type `fixed`, which means *wheel_1*
has a fixed connection with *base_link*, with all degrees-of-freedom locked.

  ```XML
  <joint name="base_to_wheel1" type="fixed">
      <parent link="base_link"/>
      <child link="wheel_1"/>
      <origin xyz="0 0 0"/>
  </joint>
  ```

For more information on the XML tags of URDF file, please refer to its
documentation [here](http://wiki.ros.org/urdf/XML).


#### Exercise: Missing URDF Components

You might have noticed a missing component in the robot-model shown on RViz.

Complete the model in the URDF file to add the missing component and relaunch
`display.launch` using the steps in the previous section ([Launch the robot simulation](#launch-the-robot-simulation))

Look for the solution below in case you can't complete the model.

<details>
<summary><b>Solution</b>: Missing URDF components</summary>

<br>&#60;link&#62; `"wheel_4"` to add a fourth wheel to the model.

```XML
<link name="wheel_4">
    <visual>
        <geometry>
            <cylinder length="0.05" radius="0.05"/>
        </geometry>
        <origin rpy="0 1.5 0" xyz="-0.1 -0.1 0"/>
        <material name="black"/>
    </visual>
</link>
```

<br>&#60;joint&#62; `"base_to_wheel4"` to connect `wheel_4` to the robot's `base_link`.

```XML
<joint name="base_to_wheel4" type="fixed">
    <parent link="base_link"/>
    <child link="wheel_4"/>
    <origin xyz="0 0 0"/>
</joint>
```
</details>

#### Adding collision and inertial properties

We finish our (box) robot description by adding `<collision>` and `<inertial>` properties
to its links. These properties are required when we run the simulation in an
environment supporting a physics engine e.g. Gazebo.

A complete description of the robot including these properties are shown in [`robot1.urdf`][robot1-urdf] 
in the [`src/robot_description/urdf`](src/robot_description/urdf) directory.

**Notes**:
* Notice the `<collision>` and `<inertial>` tags for each `<link>` element in
`robot1.urdf`.
* Take note of [`robot1.xacro`][robot1-xacro] in the [urdf/](src/robot_description/urdf) directory. `.xacro` files provide an efficient
way to reduce the size and complexity of URDF files by [using constants, simple math
expressions, and macros](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File).

For more details on collision and inertial properties, go through their the tutorial: 
["Adding Physical and Collision Properties to a URDF Model"](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model)


#### Details on simulating modelled robots

Earlier we used two packages in our launch file to spin up our modelled-robot:
[`joint_state_publisher`][ros-joint-state-publisher] and
[`robot_state_publisher`][ros-robot-state-publisher].

Few notes on these packages below:
* [`joint_state_publisher`][ros-joint-state-publisher]: Publishes a robot joints' state information (position
and velocity) as read from its URDF file. This node publishes to the `/joint_states` topic.
* [`robot_state_publisher`][ros-robot-state-publisher] : Broadcasts the state of the robot to the
[TF transform](http://wiki.ros.org/tf2) library. Listens on `/joint_states` topic
and continuously publishes the relative transforms between the joints on TF, using
its internal kinematics map that tracks the joints with respect to one another.

## Sensor Integration

In this section, we'll work with a simulated model of a [Clearpath Husky robot](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/). We will first launch it in Gazebo, which will provide us the environment that the robot will interact with, and then we'll control its movements.

### Gazebo Husky Simulation

**Important Note**: Gazebo downloads and caches it's 3D assets from the internet in the background.  If you haven't launched a Gazebo world before, it may appear to hang for ten minutes or more while the assets are downloaded in the background. The download servers are slow and often fail.  To pre-download the assets, visit [this repo](https://github.com/osrf/gazebo_models) and download the assets (Hint: Use "Download ZIP"). To install, unzip the files into `~/.gazebo/models/`.

To launch a simulated world and robot, run these commands at the same time in separate terminals:

```bash
# Launches Husky with a SICK LMS1XX lidar and and IMU, in an empty world within Gazebo.
export HUSKY_LMS1XX_ENABLED=1
roslaunch husky_gazebo husky_empty_world.launch
```
```bash
# Launches the RViz window showing Husky model and the measurements received by its lidar sensor.
roslaunch husky_viz view_robot.launch
```

Gazebo and RViz windows should appear similar to these:

![Gazebo Husky](./resources/images/gazebo_husky.png)

![RViz Husky](./resources/images/rviz_husky.png)

**Exercises**:
* Add objects (e.g. a box) in Gazebo and view its lidar scan lines on RViz.
* Try a Velodyne VLP-16 lidar 
  * See the following documentation: [Customize Husky Configuration](https://www.clearpathrobotics.com/assets/guides/noetic/husky/CustomizeHuskyConfig.html)
  * **Note**: The VLP-16 will publish under `PointCloud2`. Is it subscribed to the correct topic?
* Try loading the Husky in a more complex world

  ```bash
  # Look to the "important note" below if this doesn't load
  roslaunch husky_gazebo husky_playpen.launch
  ```

#### Exercise: Viewing TF tree and topics map

It is (always) recommended to view the TF tree and the map of topics that are
active. Make use of `tf2_tools` (or `rqt_tf_tree`) and `rqt_graph` package's commands to view this
information.

<details>
<summary>Solution: Viewing TF tree</summary>

Either 
1. Run view_frames node.

  ```bash
  rosrun tf2_tools view_frames.py
  ```

2. And view the generated TF tree.

  ```bash
  evince frames.pdf
  ``` 

Or, Run the `rqt_tf_tree` package:
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

</details>

<details>
<summary>Solution: Viewing the ROS node graph</summary>

Run the `rqt_graph` package to view the nodes and topics.
```bash
rosrun rqt_graph rqt_graph
```

</details>

### Controlling Husky

From the above exercise on viewing topics' map, you might have noticed that Husky
simulation has a `/husky_velocity_controller/cmd_vel` namespace topic published
by `/twist_mux` node and subscribed by `gazebo` node. 

We will make use of this topic to publish `geometry_msgs/Twist` messages to make Husky move.

```bash
rostopic pub -r 10 /husky_velocity_controller/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

#### Exercise: Making Husky move in circles

In this exercise you'll write a ROS node that commands the simulated Husky to
move in a circular path. Make use of the  [husky_controller](src/husky_controller) package in this repository. 
* The ROS node will need to publish messages to the `/husky_velocity_controller/cmd_vel` topic.
  Use either the `circle_driver.cpp` or `circle_driver_python.py` to get started with this task.
* **Note**: Make sure to compile and source the `devel/setup.bash` directory for your node to be included in the _path_.

<details>
<summary> Solution: Making Husky move in circles </summary>
<br>
Find a sample way to achieve this task in
<a href="resources/solutions/circle_driver_solution.md">circle_driver_solution.md</a>

</details>

### Stretch Goal : Stopping Husky in front of an object

1. Add an object on Gazebo like a box in front of Husky so that Husky stops when
it is less than 5 metres away from the box.

  * Make use of the logic in `circle_driver` above that publishes commands to
  Husky's `/husky_velocity_controller/cmd_vel` topic.

  * A ROS node that subscribes to lidar scans, observes the distance from an
  object, and sends commands similar to `circle_driver` node.

2. Find a way to integrate Kinect camera onto the simulated Husky. Going through
its [source](https://github.com/husky/husky.git) might give you few pointers.

### References

1. URDF XML Specification: http://wiki.ros.org/urdf/XML
2. Collision and Inertial properties: (http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
3. Xacro file format: http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File

[robot1-urdf]: src/robot_description/urdf/robot1.urdf
[robot1-xacro]: src/robot_description/urdf/robot1.xacro

[ros-joint-state-publisher]: http://wiki.ros.org/joint_state_publisher
[ros-robot-state-publisher]: http://wiki.ros.org/robot_state_publisher
[ros-urdf-tutorial]: https://wiki.ros.org/urdf_tutorial

[urdf]: http://wiki.ros.org/urdf/XML/model
[urdf-link]: http://wiki.ros.org/urdf/XML/link
[urdf-joint]: http://wiki.ros.org/urdf/XML/joint
