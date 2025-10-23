# ROS2 Tutorials

Step-by-step tutorials to guide you through ROS2 concepts and implementations.

## Tutorial Series

### Beginner Tutorials
1. [Getting Started with ROS2](#tutorial-1-getting-started)
2. [Understanding Nodes](#tutorial-2-nodes)
3. [Topics and Messages](#tutorial-3-topics-and-messages)
4. [Services](#tutorial-4-services)
5. [Parameters](#tutorial-5-parameters)

### Intermediate Tutorials
6. [Creating Custom Messages](#tutorial-6-custom-messages)
7. [Launch Files](#tutorial-7-launch-files)
8. [Actions](#tutorial-8-actions)
9. [Working with TF2](#tutorial-9-tf2)
10. [Building Packages with Colcon](#tutorial-10-colcon)

### Advanced Tutorials
11. [Lifecycle Nodes](#tutorial-11-lifecycle)
12. [Components and Composition](#tutorial-12-components)
13. [Real-Time and QoS](#tutorial-13-realtime-qos)
14. [Multi-Robot Systems](#tutorial-14-multi-robot)
15. [ROS2 Security](#tutorial-15-security)

---

## Tutorial 1: Getting Started

### Prerequisites
- Ubuntu 22.04 (or compatible OS)
- Basic terminal knowledge
- Python or C++ understanding

### Step 1: Install ROS2
Follow the [installation guide](../README.md#installation) in the main README.

### Step 2: Source ROS2
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 3: Test Installation
```bash
# Run demo nodes
ros2 run demo_nodes_cpp talker
```

In another terminal:
```bash
ros2 run demo_nodes_cpp listener
```

### Step 4: Explore ROS2 CLI
```bash
# List running nodes
ros2 node list

# List topics
ros2 topic list

# Get topic info
ros2 topic info /chatter

# Echo topic
ros2 topic echo /chatter
```

### What You Learned:
✓ How to source ROS2
✓ Running demo nodes
✓ Using ROS2 command-line tools
✓ Understanding topics and nodes

---

## Tutorial 2: Understanding Nodes

### What is a Node?
A node is a process that performs computation. Robots typically have many nodes working together.

### Creating Your First Node

#### Python Node:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my first node!')
        self.create_timer(1.0, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer callback: {self.counter}')

def main():
    rclpy.init()
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Concepts:
- **Node Initialization:** `super().__init__('node_name')`
- **Logging:** `self.get_logger().info()`
- **Timers:** `self.create_timer(period, callback)`
- **Spinning:** `rclpy.spin(node)` - keeps node running

### Exercise:
1. Create a node that prints "Hello" every 0.5 seconds
2. Add a counter that increments with each print
3. Make the node stop after 10 prints

---

## Tutorial 3: Topics and Messages

### Understanding Topics
Topics are named buses for nodes to exchange messages.

### Publisher Example:
```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
    
    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

### Subscriber Example:
```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Message Types:
Common standard messages:
- `std_msgs/String` - Text messages
- `std_msgs/Int32` - Integer values
- `geometry_msgs/Twist` - Velocity commands
- `sensor_msgs/Image` - Camera images
- `sensor_msgs/LaserScan` - LiDAR data

### Exercise:
1. Create a publisher that sends random numbers
2. Create a subscriber that calculates running average
3. Visualize data using `rqt_plot`

---

## Tutorial 4: Services

### Understanding Services
Services provide synchronous request-response communication.

### Service Server:
```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
    
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client:
```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

### Testing:
```bash
# Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

### Exercise:
1. Create a service that multiplies two numbers
2. Add error handling for division by zero
3. Create a calculator service with multiple operations

---

## Tutorial 5: Parameters

### What are Parameters?
Parameters are configuration values for nodes that can be changed at runtime.

### Using Parameters:
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('debug', False)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.debug = self.get_parameter('debug').value
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                if param.value < 0 or param.value > 10:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)
```

### Parameter Commands:
```bash
# Get parameter
ros2 param get /parameter_node robot_name

# Set parameter
ros2 param set /parameter_node max_speed 2.5

# List parameters
ros2 param list
```

### Parameter File (config.yaml):
```yaml
parameter_node:
  ros__parameters:
    robot_name: "my_robot"
    max_speed: 3.0
    debug: true
```

Run with parameters:
```bash
ros2 run my_package parameter_node --ros-args --params-file config.yaml
```

---

## Tutorial 6: Creating Custom Messages

### Step 1: Create Message Definition
Create `msg/PersonInfo.msg`:
```
string name
uint8 age
float32 height
```

### Step 2: Update package.xml
```xml
<depend>rosidl_default_generators</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 3: Update CMakeLists.txt
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonInfo.msg"
)
```

### Step 4: Build and Use
```bash
colcon build --packages-select my_package
source install/setup.bash

# Use in Python
from my_package.msg import PersonInfo

msg = PersonInfo()
msg.name = "John"
msg.age = 30
msg.height = 1.75
```

---

## Tutorial 7: Launch Files

### Creating a Launch File
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='talker_node',
            parameters=[{'topic_name': 'custom_topic'}]
        ),
        Node(
            package='my_package',
            executable='listener',
            name='listener_node',
            remappings=[('input_topic', 'custom_topic')]
        ),
    ])
```

### Running Launch Files:
```bash
ros2 launch my_package my_launch_file.launch.py
```

### Advanced Launch Features:
- Conditional launching
- Including other launch files
- Launch arguments
- Event handlers
- Namespace configuration

---

## Next Steps

After completing these tutorials:
1. Work through the [assignments](../assignments/)
2. Study the [examples](../examples/)
3. Build your own project
4. Explore advanced topics

## Additional Resources

- [Official ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Design Documents](https://design.ros2.org/)
- [ROS Discourse Community](https://discourse.ros.org/)
- [ROS2 GitHub](https://github.com/ros2)

Happy learning! 🎓🤖
