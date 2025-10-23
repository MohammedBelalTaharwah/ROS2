# Beginner ROS2 Assignments

This directory contains beginner-level assignments to help you learn the fundamentals of ROS2.

## Assignment List

1. [Assignment 1: ROS2 Environment Setup](#assignment-1-ros2-environment-setup)
2. [Assignment 2: First ROS2 Node](#assignment-2-first-ros2-node)
3. [Assignment 3: Publisher and Subscriber](#assignment-3-publisher-and-subscriber)
4. [Assignment 4: Services](#assignment-4-services)
5. [Assignment 5: Parameters](#assignment-5-parameters)

---

## Assignment 1: ROS2 Environment Setup

**Objective:** Set up your ROS2 development environment and verify the installation.

### Tasks:
1. Install ROS2 Humble on your system
2. Set up your workspace
3. Verify installation by running demo nodes

### Steps:
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run talker demo
ros2 run demo_nodes_cpp talker

# In another terminal, run listener
ros2 run demo_nodes_cpp listener
```

### Expected Outcome:
- The talker publishes messages
- The listener receives and prints messages
- Understanding of ROS2 sourcing

### Questions:
1. What is the purpose of sourcing ROS2?
2. What command shows all active nodes?
3. How do you list all available topics?

---

## Assignment 2: First ROS2 Node

**Objective:** Create your first ROS2 node in Python and C++.

### Tasks:
1. Create a workspace and package
2. Write a simple "Hello World" node in Python
3. Write a simple "Hello World" node in C++
4. Build and run both nodes

### Python Example Structure:
```python
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello World from ROS2!')
        
def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Deliverables:
- Working Python node
- Working C++ node
- Package with proper setup files
- Documentation of your process

---

## Assignment 3: Publisher and Subscriber

**Objective:** Understand ROS2 communication through topics.

### Tasks:
1. Create a publisher node that publishes String messages
2. Create a subscriber node that receives and prints messages
3. Publish at different frequencies (1Hz, 5Hz, 10Hz)
4. Use custom message types

### Requirements:
- Publisher publishes "Hello ROS2: [count]" every second
- Subscriber logs received messages
- Both nodes in the same package
- Proper error handling

### Testing:
```bash
# Terminal 1
ros2 run your_package publisher_node

# Terminal 2
ros2 run your_package subscriber_node

# Terminal 3 - Monitor topics
ros2 topic echo /your_topic
```

### Bonus Challenge:
- Add command-line arguments to change publish rate
- Implement QoS (Quality of Service) settings

---

## Assignment 4: Services

**Objective:** Learn synchronous communication using ROS2 services.

### Tasks:
1. Create a service server that adds two integers
2. Create a service client that requests the addition
3. Handle service call failures gracefully
4. Create a custom service definition

### Service Definition Example:
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Expected Functionality:
- Server waits for service calls
- Client sends two numbers
- Server computes and returns sum
- Both nodes log their actions

### Testing Commands:
```bash
# List services
ros2 service list

# Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

---

## Assignment 5: Parameters

**Objective:** Learn to use and manage ROS2 parameters.

### Tasks:
1. Create a node with configurable parameters
2. Read parameters from a YAML file
3. Use parameter callbacks for dynamic updates
4. Set parameters via command line and launch files

### Parameter Examples:
- String: robot_name
- Integer: max_speed
- Double: update_rate
- Boolean: debug_mode

### YAML Configuration:
```yaml
/**:
  ros__parameters:
    robot_name: "my_robot"
    max_speed: 100
    update_rate: 10.0
    debug_mode: true
```

### Testing:
```bash
# Get parameter value
ros2 param get /your_node robot_name

# Set parameter value
ros2 param set /your_node max_speed 150

# List all parameters
ros2 param list
```

---

## Submission Guidelines

For each assignment, submit:
1. Source code with comments
2. Package configuration files (package.xml, CMakeLists.txt/setup.py)
3. README with build and run instructions
4. Screenshots/logs of successful execution
5. Answers to assignment questions

## Evaluation Criteria

- Code quality and organization
- Proper error handling
- Following ROS2 best practices
- Documentation completeness
- Functionality correctness

## Additional Resources

- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Python API](https://docs.ros2.org/latest/api/rclpy/)
- [ROS2 C++ API](https://docs.ros2.org/latest/api/rclcpp/)
- [ROS2 Concepts](https://docs.ros.org/en/humble/Concepts.html)

Good luck with your assignments! 🚀
