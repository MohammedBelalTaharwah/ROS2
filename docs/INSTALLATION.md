# ROS2 Installation Guide

Complete guide for installing ROS2 on various platforms.

## Table of Contents

- [System Requirements](#system-requirements)
- [Ubuntu Installation](#ubuntu-installation)
- [Windows Installation](#windows-installation)
- [macOS Installation](#macos-installation)
- [Docker Installation](#docker-installation)
- [Troubleshooting](#troubleshooting)

---

## System Requirements

### Minimum Requirements:
- **RAM:** 4 GB (8 GB recommended)
- **Storage:** 10 GB free space
- **Processor:** x86_64 or ARM64 architecture
- **OS:** Ubuntu 20.04/22.04, Windows 10/11, macOS 10.14+

### Recommended Setup:
- **RAM:** 16 GB
- **Storage:** 50 GB SSD
- **GPU:** For simulation and vision tasks
- **Network:** Stable internet connection

---

## Ubuntu Installation

### Ubuntu 22.04 (Humble Hawksbill) - Recommended

#### Step 1: Set Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### Step 2: Setup Sources
```bash
# Install required tools
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Step 3: Install ROS2 Packages
```bash
# Update package index
sudo apt update
sudo apt upgrade

# Install ROS2 Desktop (Recommended)
sudo apt install ros-humble-desktop

# OR Install ROS2 Base (Minimal)
# sudo apt install ros-humble-ros-base

# Install development tools
sudo apt install ros-dev-tools
```

#### Step 4: Environment Setup
```bash
# Add to .bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

#### Step 5: Install Additional Tools
```bash
# Install colcon build tool
sudo apt install python3-colcon-common-extensions

# Install ROS2 visualization tools
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rqt*

# Install Gazebo (Simulation)
sudo apt install ros-humble-gazebo-ros-pkgs

# Install Navigation stack
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Install SLAM toolbox
sudo apt install ros-humble-slam-toolbox
```

### Ubuntu 20.04 (Foxy Fitzroy)
For Ubuntu 20.04, replace `humble` with `foxy` in all commands.

---

## Windows Installation

### Prerequisites:
- Windows 10/11 (64-bit)
- Visual Studio 2019 or later
- Chocolatey package manager

#### Step 1: Install Chocolatey
Open PowerShell as Administrator:
```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

#### Step 2: Install Dependencies
```powershell
choco install -y python --version=3.8.3
choco install -y vcredist2013 vcredist140
choco install -y cmake
choco install -y git
```

#### Step 3: Install ROS2
Download the latest ROS2 Windows binary from:
https://github.com/ros2/ros2/releases

Extract to `C:\dev\ros2_humble\`

#### Step 4: Setup Environment
Create a setup script `setup_ros2.bat`:
```batch
@echo off
call C:\dev\ros2_humble\local_setup.bat
```

Run this script before using ROS2.

---

## macOS Installation

### Using Homebrew

#### Step 1: Install Homebrew
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### Step 2: Install Dependencies
```bash
brew install python@3.10
brew install cmake
brew install opencv
brew install asio tinyxml2
```

#### Step 3: Install ROS2 from Source
```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

# Install dependencies
pip3 install -U colcon-common-extensions vcstool

# Build
colcon build --symlink-install
```

#### Step 4: Source ROS2
```bash
source ~/ros2_humble/install/setup.bash
echo "source ~/ros2_humble/install/setup.bash" >> ~/.zshrc
```

---

## Docker Installation

### Using Official ROS2 Docker Images

#### Step 1: Install Docker
Follow instructions at: https://docs.docker.com/get-docker/

#### Step 2: Pull ROS2 Image
```bash
docker pull ros:humble
```

#### Step 3: Run Container
```bash
# Basic container
docker run -it ros:humble

# With GUI support (Linux)
docker run -it \
  --net=host \
  --env="DISPLAY" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  ros:humble
```

#### Step 4: Create Custom Dockerfile
```dockerfile
FROM ros:humble

# Install additional packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
```

Build and run:
```bash
docker build -t my-ros2-image .
docker run -it my-ros2-image
```

---

## Post-Installation Setup

### Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Test Installation
```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener
ros2 run demo_nodes_cpp listener
```

### Setup Aliases (Optional)
Add to `~/.bashrc`:
```bash
# ROS2 Aliases
alias rb='cd ~/ros2_ws && colcon build && source install/setup.bash'
alias rs='source ~/ros2_ws/install/setup.bash'
alias rcd='cd ~/ros2_ws/src'

# Useful shortcuts
alias rn='ros2 node list'
alias rt='ros2 topic list'
alias rs='ros2 service list'
```

---

## Troubleshooting

### Common Issues

#### 1. "ros2: command not found"
**Solution:** Source the setup file
```bash
source /opt/ros/humble/setup.bash
```

#### 2. "No module named 'rclpy'"
**Solution:** Reinstall ROS2 Python packages
```bash
sudo apt install --reinstall ros-humble-rclpy
```

#### 3. Colcon build fails
**Solution:** Install missing dependencies
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

#### 4. Topics not visible between nodes
**Solution:** Check ROS_DOMAIN_ID
```bash
export ROS_DOMAIN_ID=0  # Use same ID for all nodes
```

#### 5. Performance issues
**Solution:** Adjust DDS configuration
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Getting Help

- **ROS Answers:** https://answers.ros.org/
- **ROS Discourse:** https://discourse.ros.org/
- **GitHub Issues:** https://github.com/ros2/ros2/issues
- **Documentation:** https://docs.ros.org/en/humble/

---

## Verification Checklist

After installation, verify:
- [ ] `ros2 --version` shows correct version
- [ ] Demo nodes run successfully
- [ ] `colcon build` works in workspace
- [ ] `rviz2` launches (if desktop installed)
- [ ] Topics and nodes are discoverable
- [ ] Python and C++ examples compile

---

## Uninstallation

### Ubuntu
```bash
sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### Windows
Delete the installation directory and remove environment variables.

### macOS
```bash
rm -rf ~/ros2_humble
```

---

## Next Steps

After successful installation:
1. Complete the [tutorials](../tutorials/)
2. Try the [examples](../examples/)
3. Start with [beginner assignments](../assignments/beginner/)

## Updates and Upgrades

Keep ROS2 updated:
```bash
sudo apt update
sudo apt upgrade
```

For major version upgrades, refer to the official migration guides.

---

**Installation complete! Ready to start learning ROS2! 🚀**
