# Hands-On: ROS Subscriber for `LaserScan` Topic to Extract Front, Right, and Left Data

In ROS, the `LaserScan` topic contains data from a laser scanner (such as LiDAR), which provides distance measurements from obstacles or objects around the robot. This hands-on guide explains how to create a ROS Subscriber in Python to read data from the `/scan` topic and extract specific readings for the front, right, and left of the robot.

## Prerequisites

- ROS 1 installed on your system (e.g., ROS Noetic).
- A ROS workspace set up with a source directory for ROS packages.
- A robot simulation environment like Gazebo with a robot model that publishes to the `/scan` topic.
- Basic knowledge of Python and ROS concepts.

## Step 1: Create a ROS Package

First, create a ROS package where your subscriber script will reside.

1. **Navigate to Your Workspace Source Directory**:

   ```bash
   cd ~/catkin_ws/src
   ```

2. **Create a New Package**:

   ```bash
   catkin_create_pkg my_laserscan_subscriber rospy sensor_msgs roscpp
   ```

   This command creates a package named `my_laserscan_subscriber` with dependencies on `rospy`, `sensor_msgs`, and `roscpp`.

3. **Build the Workspace**:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

   This compiles your workspace and registers the new package with ROS.

## Step 2: Create a ROS Subscriber Node in Python

Create a Python script that subscribes to the `/scan` topic and retrieves `LaserScan` data, focusing on the front, right, and left readings.

1. **Create the Python Script**:
   Navigate to the `src` directory of your package and create a new Python script.

   ```bash
   cd ~/catkin_ws/src/my_laserscan_subscriber
   touch laserscan_subscriber.py
   chmod +x laserscan_subscriber.py  # Make the script executable
   ```

2. **Write the Subscriber Code**:
   Open `laserscan_subscriber.py` in a text editor and add the following code:

   ```python
   #!/usr/bin/env python3
   import rospy
   from sensor_msgs.msg import LaserScan

   def laserscan_callback(data):
       # Get the index for front, right, and left
       # Assuming a 180-degree LiDAR with a 0-degree angle at the front of the robot
       num_ranges = len(data.ranges)
       front_idx = num_ranges // 2
       right_idx = front_idx // 2
       left_idx = front_idx + right_idx

       # Get the range readings for front, right, and left
       front_range = data.ranges[front_idx]
       right_range = data.ranges[right_idx]
       left_range = data.ranges[left_idx]

       # Print the readings
       rospy.loginfo(f"Front Range: {front_range:.2f} m")
       rospy.loginfo(f"Right Range: {right_range:.2f} m")
       rospy.loginfo(f"Left Range: {left_range:.2f} m")

   def laserscan_subscriber():
       # Initialize the ROS node
       rospy.init_node('laserscan_subscriber', anonymous=True)

       # Subscribe to the /scan topic
       rospy.Subscriber('/scan', LaserScan, laserscan_callback)

       # Keep the node alive until ROS is shut down
       rospy.spin()

   if __name__ == '__main__':
       laserscan_subscriber()
   ```

This script creates a ROS node named `laserscan_subscriber` that subscribes to the `/scan` topic. The `LaserScan` message type provides an array of distance readings. The code extracts readings from the middle (front), one-quarter (right), and three-quarters (left) positions in the `ranges` array, assuming a 180-degree laser scanner.

## Step 3: Run the Subscriber Node

To test the subscriber, ensure the ROS core is running, and that there's a robot or simulation publishing to the `/scan` topic.

1. **Start the ROS Core**:
   If it's not already running, start the ROS master.

   ```bash
   roscore
   ```

2. **Run the Subscriber Node**:
   Open a new terminal, source your workspace setup script, and run your Python script.

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun my_laserscan_subscriber laserscan_subscriber.py
   ```

   You should see log messages indicating the front, right, and left range readings from the `/scan` topic.

## Conclusion

You have successfully created a ROS Subscriber in Python that subscribes to the `LaserScan` topic and retrieves data for the front, right, and left directions. This subscriber is useful for obtaining distance measurements from a LiDAR or similar sensor, and can be extended to implement robot navigation, obstacle avoidance, and other advanced behaviors.
