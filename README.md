# Robotics Development Environment Setup

This guide provides instructions for setting up a ROS 2 and Gazebo environment for your robotics projects. Please follow the option that best suits your operating system and hardware.

## Introduction to ROS 2 and Gazebo

### What is ROS 2?
The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the second generation of ROS, and it is widely used in both academia and industry for robotics research and development. It provides a message-passing interface that allows different parts of a robot's software to communicate with each other.

### What is Gazebo?
Gazebo is a powerful 3D robotics simulator that allows you to test and validate your robot's software in a virtual environment before deploying it on a physical robot. It provides realistic physics simulation, a variety of sensors, and a graphical interface to visualize your robot and its environment.

### Why ROS 2 Jazzy and Gazebo Harmonic?
ROS 2 and Gazebo releases are tightly coupled with specific versions of Ubuntu. We will be using ROS 2 Jazzy Jellyfish and Gazebo Harmonic, which are the recommended versions for Ubuntu 24.04. This ensures compatibility with Ubuntu 24.04 and access to the latest features and bug fixes.

## Option 1: Native Ubuntu 24.04 Installation

This is the recommended approach if you have a computer running Ubuntu 24.04 with sufficient resources (recommended: 4-core/8-thread CPU, 16GB RAM). The current setup is only supported for the bash shell.

1.  **Install ROS 2:**
    Follow the official ROS 2 Jazzy installation guide:
    [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), specifically:

    ```bash
    # 1. Set Locale
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

    # 2. Add the ROS 2 APT repository
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivatives use $UBUNTU_CODENAME
    sudo dpkg -i /tmp/ros2-apt-source.deb

    # 3. Install ROS 2 packages
    sudo apt update
    sudo apt install ros-jazzy-desktop # Includes GUI tools and simulators

    # 4. Source the setup script
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc # Automatically configure the ROS 2 environment for every new terminal
    ```

2.  **Install Gazebo:**
    Use the official pairing of Gazebo and ROS2:
    ```bash
    sudo apt-get install ros-jazzy-ros-gz
    ```
    **Make sure you have sourced the ros2 setup script following the previous instruction.**
    You can also install the standalone binary version of Gazebo if preferred; see: [Official Gazebo with ROS installation guide](https://gazebosim.org/docs/latest/ros_installation/).

3.  **Install Franka ROS2 Workspace:**
    To install the Franka workspace, follow the installation instructions from the `franka_ros2` repository. The setup script in Option 3 contains the necessary commands.
    <!-- https://github.com/frankarobotics/franka_ros2 -->
    [https://github.com/frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2)


    Alternatively, you can use the provided scripts to automate the installation of ROS2 and dependencies and the Franka ROS2 packages. Make sure to review the scripts before running them.

    Clone this repository or download the scripts.

    For ROS2 installation and dependencies, run:
    ```bash
    chmod +x ros2_setup.sh
    ./ros2_setup.sh
    ```

    For Franka ROS2 packages, run:
    ```bash
    chmod +x franka_setup.sh
    ./franka_setup.sh
    ```

## Option 2: Docker for macOS, Windows Subsystem for Linux (WSL), and other Linux distributions

If you are using macOS, WSL, or a non-Ubuntu Linux distribution, you can use a pre-configured Docker container. This method requires a powerful workstation; otherwise the simulation will be laggy.

### What is Docker?
Docker is a platform that allows you to run applications in isolated environments called containers. We use it to provide a consistent and pre-configured development environment with all the necessary tools, without affecting your own computer's system. For AI and Computer Science students, learning to use Docker is a very valuable skill.

1.  **Install Docker:**
    Download and install Docker for your operating system:
    -   **macOS:** [https://docs.docker.com/desktop/setup/install/mac-install/](https://docs.docker.com/desktop/setup/install/mac-install/)
    -   **Windows:** [https://docs.docker.com/desktop/setup/install/windows-install/](https://docs.docker.com/desktop/setup/install/windows-install/)
    -   **Linux:** [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)

2. **Clean up old images (if any):**
   If you have previously run the Docker container, it is recommended to remove any existing images of `storkslab/ros2-jazzy-franka` to avoid conflicts with previous versions. You can do this by executing the following command in your terminal:
   ```bash
    docker rmi storkslab/ros2-jazzy-franka --force
    # or
    docker image pull storkslab/ros2-jazzy-franka:latest
   ```

3.  **Run the docker container:**
    Open a terminal and run the following command. If you are not familiar with the terminal, it is a good idea to refresh your bash/terminal skills.
    ```bash
    docker run --rm -p 8888:8888 storkslab/ros2-jazzy-franka:latest jupyter server
    ```
    This command will download the Docker image and start a container that removes itself on shutdown (`--rm`). Any modifications to the container's filesystem outside of the persisted volume will be lost when the container is removed. The container runs a Jupyter Lab instance with a VNC server, giving you access to a full desktop environment.
    
    To keep your progress, you can also run the container in detached mode by adding the `-d` flag:
    ```bash
    docker run -d -p 8888:8888 storkslab/ros2-jazzy-franka:latest jupyter server
    ```
    and use stop/start commands to manage the container. You can find more information about managing Docker containers in the [Docker documentation](https://docs.docker.com/get-started/overview/).

4.  **Access the environment:**
    -   Open your web browser and navigate to `https://localhost:8888/lab`. It will ask for a password, which is `robotics2025` by default.
    -   Make sure you use `https` if the server was configured with TLS; otherwise use the appropriate protocol shown by the container logs.
    -   You may see a security warning. Click "Advanced" and "Proceed anyway" if you trust the connection.
    -   The Jupyter Lab interface will open. On the launcher, click on the "Desktop" icon.
    -   A new browser tab will open with a full Linux desktop environment. Congratulations, you are in!

## Option 3: Google Cloud Platform (GCP) Virtual Machine

If your computer is not powerful enough to run the simulation natively or via Docker, you can use a virtual machine on GCP and run Docker there.

### Step 1: Create a GCP Account & Project
-   Sign up for a GCP account at [https://cloud.google.com/](https://cloud.google.com/).
-   If you have a `@student.vu.nl` email, you may be eligible for $300 in free credits.
-   Create a new project for this course, and enable billing.

### Step 2: Create a Virtual Machine
-   In the GCP Console, navigate to **Compute Engine** > **VM instances**.
-   Click **Create Instance**.
-   Configure your VM with the following settings:
    - **Machine configuration:**
        -   **Name:** Create your own instance name (e.g., `ros2-workbench`).
        -   **Region:** Choose a region in Europe for low latency (e.g., `europe-west4` in the Netherlands).
        -   **Series:** C4
        -   **Machine type:** `c4-highcpu-8` (8 vCPU, 16 GB memory).
    -   **OS and Storage:**
        -   Click **Change**.
        -   **Operating system:** Container-Optimized OS (COS)
        -   **Version:**
            - Container-Optimized OS 121-18867.199.52 LTS (x86/64)
        -   **Boot disk type:** Balanced Persistent Disk
        -   **Size (GB):** 50
        -   Click **Select**.
    -   **Networking:**
        -   Check **Allow HTTP traffic** and **Allow HTTPS traffic**.
        -   **Network tags:**
            -   Add the tag `jupyter-lab`.
-   Click **Create**. The VM will be deployed in about a minute. Note its external IP.

### Step 3: Configure Firewall Rule
-   In the GCP console, open the **Cloud Shell** (top-right `>_` icon).
-   Run the following command in the Cloud Shell terminal to allow access to JupyterLab on port 8888.
    ```bash
    gcloud compute firewall-rules create allow-jupyter-lab-tcp-8888 \
        --allow=tcp:8888 \
        --source-ranges=0.0.0.0/0 \
        --target-tags=jupyter-lab \
        --description="Allow ingress TCP traffic on port 8888 for Jupyter Lab instances"
    ```
    This rule applies to any VM with the `jupyter-lab` tag.

### Step 4: Connect to the VM and Run Docker Container
-   Connect to your VM using SSH. You can click the **SSH** button next to your instance in the GCP Console.

-   **Clean up old images (if any):**
    If you have previously run the Docker container, it is recommended to remove any existing images of `storkslab/ros2-jazzy-franka` to avoid conflicts with previous versions. You can do this by executing the following command in your terminal:
    ```bash
    docker rmi storkslab/ros2-jazzy-franka --force
    # or
    docker image pull storkslab/ros2-jazzy-franka:latest
    ```
-   In the SSH terminal, run the following command to start the Docker container (same instruction as in Option 2):
    ```bash
    docker run --rm -p 8888:8888 storkslab/ros2-jazzy-franka:latest jupyter server
    ```
    This command will download the Docker image and start a container that removes itself on shutdown (`--rm`). Any modifications to the container's filesystem outside of the persisted volume will be lost when the container is removed. The container runs a Jupyter Lab instance with a VNC server, giving you access to a full desktop environment.
    
    To keep your progress, you can also run the container in detached mode by adding the `-d` flag:
    ```bash
    docker run -d -p 8888:8888 storkslab/ros2-jazzy-franka:latest jupyter server
    ```
    and use stop/start commands to manage the container. You can find more information about managing Docker containers in the [Docker documentation](https://docs.docker.com/get-started/overview/).


### Step 5: Access the Environment
-   In your local web browser, navigate to `https://<your_vm_external_ip>:8888/lab`.
-   You may see a security warning. Click "Advanced" and "Proceed anyway".
-   Enter the password `robotics2025` when prompted.

Congrats! Your setup is complete!

### Important notes about use of VM.
- **Shutting down the VM:** When you are done, shut down the VM from the GCP Console to avoid incurring costs. You can restart it later.
- **Data persistence:** The Docker volume `ros2_ws` is used to persist your work. Any files you create or modify in `/home/jovyan/ros2_ws` inside the container will be saved to the `ros2_ws` volume on the VM, and will persist even after the container is stopped or removed. Best practice is to keep all your work inside this directory, or another attached volume, and to delete the container after use (`--rm`). Any modifications to the container's filesystem outside of this volume will be lost when the container is removed.
- **Costs:** Running a VM incurs costs based on usage. You have 300 USD in free credits if you signed up with a student email. Monitor your usage in the GCP Console to avoid unexpected charges.
- **Scheduled shutdown:** To avoid forgetting to shut down the VM, you can set up a scheduled shutdown using GCP's 

---

## Part 1: Run a Test Program

After setting up your environment, run a test program to visualize the Franka robot arm in Gazebo.

1.  If using a VM, open the desktop environment from the JupyterLab launcher.
2.  Open a terminal.
3.  Navigate to the Franka workspace and source the overlay:
    ```bash
    cd ~/franka_ros2_ws
    source install/setup.bash
    ```
4.  Run the launch file:
    ```bash
    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py
    ```
You should see two windows open: RViz (visualizer) and Gazebo (simulator). Note that Gazebo may take a while to load the first time, and it could be obscured by other windows. Please take a screenshot of the result for your submission.

You can close all windows and stop the program by pressing `Ctrl+C` in the terminal.

---

## Part 2: ROS 2 Publisher-Subscriber Tutorial (Python)

This tutorial, adapted from the official ROS 2 documentation, guides you through creating a simple publisher and subscriber showing the decoupled communication between a publisher and subscriber via topics managed by the ROS 2 middleware. 

### About ROS 2 Workspaces (don't run this)

A ROS2 workspace is somewhat similar to a virtual environment; it is a directory where you can work on the development of individual project packages, build, modify and install ROS 2 packages locally. You can have multiple workspaces but only one can be sourced at a time to make its packages available in your terminal environment. The building of different packages and their dependencies is handled by the build tool `colcon`.

#### The workspace structure

After being built, a standard ROS 2 workspace normally has the following directory
```bash
your_workspace/            # The workspace root
├── build/                 # (Generated by `colcon`) Contains intermediate build files
├── install/               # (Generated by `colcon`) The installation directory. This is where the built packages are installed. It contains its own `setup.bash` script.
├── log/                   # (Generated by `colcon`) Detailed build and test logs for each package.
└── src/                   # **The only directory you create manually.**
    └── your_package_1/    # Your custom packages or ones you've cloned from Git.
    └── your_package_2/
    └── ...               
```
Notice that all your source code for your packages is contained in `src/` only; here you could either create new packages (see Part 3 for details) or clone existing packages from git but you should always build from the workspace root using colcon.

#### Sourcing

Previously in the native installation guide, we have sourced the setup file directly into .bashrc so that you do not have to source it again every time a new terminal is being opened. However, one could overlay a custom workspace on top of the core ROS 2 environment by running
```bash
source ~/your_workspace/install/setup.bash
```
Now your current terminal can look for packages first in the `~/your_workspace/install/` directory and will only fall back to the core setup if a certain package could not be found in the overlay. Therefore make sure you source the custom setup file in every new terminal where you would like to use your packages but **NEVER** source multiple setup files directly into .bashrc.

### 1. Create a Package

Note: you can use either JupyterLab terminal or the desktop terminal to do this part. It is easier to use JupyterLab to copy-paste the commands.

Open a terminal and navigate to the `src` directory of a new workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Create a new Python package:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

### 2. Write the Publisher Node

Navigate into the new package directory:
```bash
cd py_pubsub/py_pubsub
```

Create a file for the publisher node (feel free to use any other text editors you prefer on your local Ubuntu system):
```bash
nano publisher_member_function.py
```

Copy and paste the following code into the file:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10) # Defines the type of topic for the communication
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Save and exit `nano`.

### 3. Write the Subscriber Node

In the same directory (`~/ros2_ws/src/py_pubsub/py_pubsub`), create the subscriber file:
```bash
nano subscriber_member_function.py
```

Copy and paste the following code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Save and exit `nano`.

### 4. Configure the Package

Navigate back to the package root (`~/ros2_ws/src/py_pubsub`).

#### a. Edit `package.xml`
Open `package.xml`.

First, update the metadata in these three existing tags:
```xml
  <description>Examples of minimal publisher/subscriber using rclpy</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>
```

Then add the following lines inside the `<package>` tag to declare dependencies and metadata.
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

Save and exit `nano`.

#### b. Edit `setup.py`
Open `setup.py` and add the entry points for your nodes inside the `entry_points` dictionary.
```python
# ... inside setup()
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
# ...
```
Also, update the maintainer, email, description, and license fields to match `package.xml`.

### 5. Build and Run

Navigate to the root of your workspace (`~/ros2_ws`) and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

#### a. Run the Publisher
Open a **new terminal**. Source the setup file and run the publisher:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub talker
```
You will see `Publishing: "Hello World: ..."` messages.

#### b. Run the Subscriber
Open a **second new terminal**. Source the setup file and run the subscriber:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub listener
```
You will see `I heard: "Hello World: ..."` messages.
Please take a screenshot of both terminals side by side for your submission.

Press `Ctrl+C` in each terminal to stop the nodes.

### (Optional) 6. Monitor the actions under the hood

You could also use ROS 2 CLI tools to inspect what is going on during this interaction. Keep the two existing terminals and open a third terminal:

- `ros2 topic echo /topic` will show you the messages being published in real-time, just like the subscriber node does.

- `ros2 node list` will show you both \minimal_publisher and \minimal_subscriber.

- `ros2 node info /minimal_publisher` will show you that it is publishing to /topic.

- `ros2 node info /minimal_subscriber` will show you that it is subscribing to /topic.

The publisher and subscriber pattern is the most fundamental communication method in ROS 2; it is the backbone of the communication system where sensors publish certain data and other nodes subscribe to it for processing and decision making.