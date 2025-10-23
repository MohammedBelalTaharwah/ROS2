# Intermediate ROS2 Assignments

This directory contains intermediate-level assignments to deepen your understanding of ROS2.

## Assignment List

1. [Assignment 1: Custom Messages and Services](#assignment-1-custom-messages-and-services)
2. [Assignment 2: Launch Files](#assignment-2-launch-files)
3. [Assignment 3: Actions](#assignment-3-actions)
4. [Assignment 4: TF2 Transformations](#assignment-4-tf2-transformations)
5. [Assignment 5: Multi-Node Communication](#assignment-5-multi-node-communication)

---

## Assignment 1: Custom Messages and Services

**Objective:** Create and use custom message and service types for specialized communication.

### Tasks:
1. Define a custom message for robot sensor data
2. Define a custom service for robot control commands
3. Create publishers/subscribers using custom messages
4. Implement service server/client with custom service

### Custom Message Example:
```
# RobotSensorData.msg
std_msgs/Header header
float64 temperature
float64 battery_voltage
int32 obstacle_distance
string status
```

### Custom Service Example:
```
# RobotControl.srv
string command
float64[] parameters
---
bool success
string message
```

### Deliverables:
- Message and service definition files
- Implementation nodes
- Documentation of message/service structure
- Test cases demonstrating usage

---

## Assignment 2: Launch Files

**Objective:** Master ROS2 launch system for managing multiple nodes and configurations.

### Tasks:
1. Create a launch file that starts multiple nodes
2. Set parameters via launch file
3. Use launch arguments for configuration
4. Implement conditional node launching
5. Create a launch file that includes other launch files

### Launch File Requirements:
- Start at least 3 different nodes
- Configure parameters for each node
- Use namespaces for node organization
- Include remapping of topics
- Add logging configuration

### Example Structure:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='your_package',
            executable='node_1',
            name='node_1',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Add more nodes...
    ])
```

### Testing:
```bash
ros2 launch your_package your_launch_file.launch.py
ros2 launch your_package your_launch_file.launch.py use_sim_time:=true
```

---

## Assignment 3: Actions

**Objective:** Implement ROS2 actions for long-running tasks with feedback.

### Tasks:
1. Define a custom action for a robot navigation task
2. Implement an action server
3. Implement an action client with goal handling
4. Provide periodic feedback during execution
5. Handle preemption and cancellation

### Custom Action Example:
```
# NavigateToGoal.action
geometry_msgs/PoseStamped target_pose
---
bool success
geometry_msgs/PoseStamped final_pose
---
float64 distance_remaining
float64 estimated_time_remaining
```

### Requirements:
- Action server accepts goal and processes it
- Server sends feedback every second
- Client can cancel goals
- Proper result handling
- Concurrent goal management

### Testing:
```bash
# List actions
ros2 action list

# Send goal from command line
ros2 action send_goal /navigate_to_goal your_interfaces/action/NavigateToGoal "{target_pose: {pose: {position: {x: 1.0, y: 2.0}}}}"
```

---

## Assignment 4: TF2 Transformations

**Objective:** Work with coordinate frame transformations using TF2.

### Tasks:
1. Broadcast static transforms between frames
2. Broadcast dynamic transforms (moving frames)
3. Listen to transforms and use them in calculations
4. Create a TF tree with multiple frames
5. Visualize the TF tree in RViz2

### Frame Hierarchy Example:
```
world
└── robot_base
    ├── laser_scanner
    └── camera
```

### Requirements:
- Publish transforms at appropriate rates (>10 Hz for dynamic)
- Handle transform exceptions properly
- Convert between different coordinate frames
- Time synchronization for historical transforms

### Useful Commands:
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo world robot_base

# Visualize in RViz2
rviz2
```

### Bonus:
- Implement a robot arm with multiple joints
- Compute forward kinematics using TF2

---

## Assignment 5: Multi-Node Communication

**Objective:** Build a complex system with multiple interacting nodes.

### Tasks:
Create a simulated robot system with:
1. Sensor node (publishes sensor data)
2. Processing node (processes sensor data, uses services)
3. Control node (receives commands, uses actions)
4. Monitor node (displays system status)
5. Configuration node (manages parameters)

### System Requirements:
- At least 5 different nodes
- Use topics for data streaming
- Use services for queries
- Use actions for commands
- Use parameters for configuration
- Implement proper namespacing
- Create a comprehensive launch file

### Communication Flow:
```
Sensor Node -> Processing Node -> Control Node
     ↓              ↓                  ↓
     └──────> Monitor Node ←───────────┘
                    ↑
              Configuration Node
```

### Quality Requirements:
- Implement QoS profiles appropriate for each communication
- Handle network failures gracefully
- Log important events
- Implement health monitoring

### Testing Scenarios:
1. Start all nodes and verify communication
2. Stop individual nodes and verify system resilience
3. Change parameters and verify system adaptation
4. Send various commands and monitor feedback

---

## Submission Guidelines

For each assignment, submit:
1. Complete source code with comments
2. Custom message/service/action definitions
3. Launch files
4. Configuration files (YAML)
5. Comprehensive README with:
   - Architecture diagram
   - Build instructions
   - Run instructions
   - Testing procedures
6. Screenshots or videos of working system
7. Test results and logs

## Evaluation Criteria

- System design and architecture
- Code quality and maintainability
- Proper use of ROS2 features
- Error handling and robustness
- Documentation quality
- Performance considerations

## Best Practices

1. **Code Organization:**
   - Separate concerns into different nodes
   - Use classes for complex logic
   - Follow PEP 8 (Python) or Google C++ Style Guide

2. **ROS2 Specific:**
   - Use appropriate QoS settings
   - Implement proper lifecycle management
   - Handle exceptions and errors
   - Use ROS2 logging (not print statements)

3. **Documentation:**
   - Document all parameters
   - Explain message/service/action definitions
   - Provide usage examples
   - Include troubleshooting tips

## Additional Resources

- [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client.html)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2.html)
- [Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

Good luck with your intermediate assignments! 🤖
