# ROS2 Learning Repository

Welcome to the ROS2 (Robot Operating System 2) Learning Repository! This repository contains comprehensive assignments, tutorials, and learning materials for mastering ROS2.

## 📚 Table of Contents

- [About ROS2](#about-ros2)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Repository Structure](#repository-structure)
- [Assignments](#assignments)
- [Tutorials](#tutorials)
- [Examples](#examples)
- [Resources](#resources)
- [Contributing](#contributing)

## 🤖 About ROS2

ROS2 is the next generation of the Robot Operating System (ROS), designed for production robotics applications. It provides libraries, tools, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Features of ROS2:
- Real-time capable
- Multi-platform support (Linux, Windows, macOS)
- Security features built-in
- Improved performance and reliability
- DDS (Data Distribution Service) middleware
- Better support for multi-robot systems

## 📋 Prerequisites

Before starting with ROS2, you should have:
- Basic understanding of Linux command line
- Programming knowledge in Python or C++
- Familiarity with basic robotics concepts (recommended)
- Understanding of Object-Oriented Programming

## 🔧 Installation

### Ubuntu 22.04 (Humble Hawksbill)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

For other platforms, see the [official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html).

## 📁 Repository Structure

```
ROS2/
├── assignments/          # Assignments organized by difficulty
│   ├── beginner/        # Basic ROS2 concepts
│   ├── intermediate/    # More complex tasks
│   └── advanced/        # Advanced ROS2 features
├── tutorials/           # Step-by-step tutorials
├── examples/            # Example code and projects
├── docs/                # Additional documentation
└── README.md           # This file
```

## 📝 Assignments

### Beginner Level
Learn the fundamentals of ROS2:
- Setting up ROS2 environment
- Creating and running nodes
- Understanding topics, services, and actions
- Working with publishers and subscribers
- Using ROS2 command-line tools

### Intermediate Level
Build upon basic knowledge:
- Creating custom messages and services
- Working with parameters
- Launch files and configurations
- TF2 transformations
- Building packages with colcon

### Advanced Level
Master advanced ROS2 concepts:
- Multi-robot systems
- Real-time performance tuning
- Security and QoS settings
- Custom DDS configurations
- Integration with hardware

## 🎓 Tutorials

Detailed step-by-step tutorials covering:
1. **ROS2 Basics** - Nodes, topics, services
2. **Package Development** - Creating and managing ROS2 packages
3. **Visualization** - Using RViz2 and rqt tools
4. **Simulation** - Gazebo integration
5. **Navigation** - Nav2 stack usage

## 💡 Examples

Practical examples demonstrating:
- Simple publisher/subscriber (Python & C++)
- Service client/server
- Action server/client
- Custom message definitions
- Launch file configurations
- Parameter management

## 📖 Resources

### Official Documentation
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Design](https://design.ros2.org/)

### Community Resources
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [ROS2 GitHub](https://github.com/ros2)

### Books and Courses
- "A Concise Introduction to Robot Programming with ROS2"
- "Programming Robots with ROS"
- Online courses on Udemy, Coursera, and edX

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Bug fixes
- New assignments
- Tutorial improvements
- Additional examples
- Documentation updates

## 📄 License

This repository is for educational purposes. Please check individual file licenses where applicable.

## 🌟 Getting Started

1. Clone this repository
2. Complete the ROS2 installation
3. Start with beginner assignments
4. Progress through tutorials
5. Build your own projects!

Happy Learning! 🚀
