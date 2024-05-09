# Hands-On: ROS Publisher for Moving a Robot Using `cmd_vel`

In ROS, the `cmd_vel` topic is commonly used to control robot movement. Messages published to this topic typically control the robot's linear and angular velocities. In this hands-on guide, you'll create a ROS Publisher in Python that publishes to the `cmd_vel` topic to make a robot move forward and then rotate.

## Prerequisites
- ROS 1 installed on your system (e.g., ROS Noetic).
- A ROS workspace set up with a source directory for ROS packages.
- A robot simulation environment like Gazebo, or a physical robot capable of using `cmd_vel`.
- Basic knowledge of Python and ROS concepts such as nodes, topics, and messages.

## Step 1: Create a ROS Package
First, create a ROS package where your publisher script will reside.

1. **Navigate to Your Workspace Source Directory**:
   ```bash
   cd ~/catkin_ws/src
   ```

2. **Create a New Package**:
   ```bash
   catkin_create_pkg my_cmd_vel_publisher rospy geometry_msgs roscpp
   ```

   This command creates a package named `my_cmd_vel_publisher` with dependencies on `rospy`, `geometry_msgs`, and `roscpp`.

3. **Build the Workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

   This compiles your workspace and registers the new package with ROS.

## Step 2: Create a ROS Publisher Node in Python
Next, create a Python script that publishes messages to the `cmd_vel` topic to control a robot's movement.

1. **Create the Python Script**:
   Navigate to the `src` directory of your package and create a new Python script.

   ```bash
   cd ~/catkin_ws/src/my_cmd_vel_publisher
   touch move_robot.py
   chmod +x move_robot.py  # Make the script executable
   ```

2. **Write the Publisher Code**:
   Open `move_robot.py` in a text editor and add the following code:

   ```python
   #!/usr/bin/env python3
   import rospy
   from geometry_msgs.msg import Twist  # Message type for controlling robot velocity

   def move_robot():
      # Initialize the ROS node
      rospy.init_node('move_robot', anonymous=True)

      # Create a publisher for the 'cmd_vel' topic
      pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

      # Create a Twist message to control linear and angular velocities
      twist = Twist()
      # Set the publishing rate (1 Hz)
      rate = rospy.Rate(1)  # 1 Hz

      while not rospy.is_shutdown():

         # Move forward with linear velocity
         twist.linear.x = 0.2  # Positive value to move forward
         twist.angular.z = 0.0  # No angular movement (no turning)

         # Publish the message to move the robot forward
         rospy.loginfo("Moving forward")
         pub.publish(twist)

         rate.sleep()

      
   if __name__ == '__main__':
      try:
         move_robot()
      except rospy.ROSInterruptException:
         pass
      
   ```

This script initializes a ROS node, creates a publisher for the `cmd_vel` topic, and sends commands to move a robot forward and then rotate. The `geometry_msgs/Twist` message type allows you to set linear and angular velocities.

## Step 3: Run the Publisher Node
To test the publisher, ensure that the ROS core is running, and that you have a robot simulation or a physical robot that listens to `cmd_vel`.

1. **Start the ROS Core**:
   If it's not already running, start the ROS master.

   ```bash
   roscore
   ```

2. **Run the Publisher Node**:
   Open a new terminal, source your workspace setup script, and run your Python script.

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun my_cmd_vel_publisher move_robot.py
   ```

   If you're using a robot simulation like Gazebo, you should see the robot move forward and then rotate. If you're using a physical robot, ensure it's safe to move and has adequate space.

## Conclusion
You have successfully created a ROS Publisher in Python that publishes messages to the `cmd_vel` topic to control robot movement. This is a basic example of using ROS to command a robot's motion. You can extend this example to implement more complex motion patterns, integrate with other sensors, or create automated behaviors.