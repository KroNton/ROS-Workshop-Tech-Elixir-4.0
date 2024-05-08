# Hands-On Guide: Creating a New ROS Workspace and Package

Creating a new workspace and package is a fundamental task in ROS development. This hands-on guide will walk you through the steps to set up a ROS workspace, create a new package, and build it. The following instructions are intended for ROS 1 with the `catkin` build system.

## Prerequisites
- ROS 1 installed on your system (e.g., ROS Noetic).
- A working terminal on your system.
- Basic understanding of Linux commands.

## Step 1: Create a New Workspace
The first step is to create a workspace directory where you'll organize your ROS packages.

1. **Create a Directory**:
   Open a terminal and create a new directory for your workspace. You can name it whatever you like, but for this guide, we'll use `my_ros_workspace`.

   ```bash
   mkdir -p ~/my_ros_workspace/src
   ```

2. **Navigate to the Workspace Directory**:
   Change into your newly created workspace directory.

   ```bash
   cd ~/my_ros_workspace
   ```

3. **Initialize the Workspace**:
   To set up the workspace for `catkin`, initialize it by running `catkin_make`. This command will create the necessary build and devel (development) directories.

   ```bash
   catkin_make
   ```

   After running this command, you should see `build/` and `devel/` directories created within your workspace. The `devel/` directory contains build artifacts and setup scripts.


## Step 2: Create a New Package
With your workspace set up, you can create a new ROS package.

1. **Navigate to the `src/` Directory**:
   This is where you'll create new ROS packages within your workspace.

   ```bash
   cd ~/my_ros_workspace/src
   ```

2. **Create a Package**:
   Use the `catkin_create_pkg` command to create a new ROS package. You need to provide a package name and list any dependencies. For this example, we'll create a package named `my_first_package` with a dependency on `std_msgs`.

   ```bash
   catkin_create_pkg my_first_package std_msgs
   ```

   This command creates a directory named `my_first_package` with the following files:
   - `package.xml`: Package metadata, including dependencies and other information.
   - `CMakeLists.txt`: Instructions for building the package.

3. **Build the Package**:
   After creating the package, go back to the workspace root and build it with `catkin_make`.

   ```bash
   cd ~/my_ros_workspace
   catkin_make
   ```

4. **Verify the Package**:
   Once the build is complete, you can use the `rospack` command to verify that the package is recognized by ROS.

   ```bash
   rospack find my_first_package
   ```

   If the command returns the package path, it means your package is correctly created and recognized by ROS.

## Conclusion
You've now created a new ROS workspace, created a ROS package, and built it. This is the foundation for any ROS project. From here, you can add ROS nodes, launch files, configuration files, and more to build complex robotic applications. Remember to source the workspace setup script every time you start a new terminal session to ensure your environment variables are correctly set.