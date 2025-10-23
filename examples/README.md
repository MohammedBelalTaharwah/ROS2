# ROS2 Examples

This directory contains practical code examples demonstrating various ROS2 concepts.

## Available Examples

### Basic Examples
1. [Simple Publisher-Subscriber (Python)](#python-publisher-subscriber)
2. [Simple Publisher-Subscriber (C++)](#cpp-publisher-subscriber)
3. [Service Client-Server](#service-example)
4. [Parameter Usage](#parameter-example)

### Intermediate Examples
5. [Action Server-Client](#action-example)
6. [Custom Messages](#custom-message-example)
7. [Launch Files](#launch-file-example)
8. [TF2 Broadcaster](#tf2-example)

### Advanced Examples
9. [Lifecycle Node](#lifecycle-example)
10. [Component](#component-example)

---

## Python Publisher-Subscriber

### Publisher (minimal_publisher.py)
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """Simple publisher that sends a message every second."""
    
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        """Callback function that publishes messages."""
        msg = String()
        msg.data = f'Hello ROS2: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber (minimal_subscriber.py)
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Simple subscriber that receives and logs messages."""
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        """Callback function for received messages."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Example:
```bash
# Terminal 1
python3 minimal_publisher.py

# Terminal 2
python3 minimal_subscriber.py
```

---

## C++ Publisher-Subscriber

### Publisher (minimal_publisher.cpp)
```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Publisher node started");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### Subscriber (minimal_subscriber.cpp)
```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscriber node started");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

---

## Service Example

### Service Server (add_two_ints_server.py)
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers."""
    
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Add Two Ints Server Ready')

    def add_two_ints_callback(self, request, response):
        """Handle service requests."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a} b={request.b}, '
            f'sending back: {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client (add_two_ints_client.py)
```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that requests addition of two numbers."""
    
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        """Send service request."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: add_two_ints_client.py <a> <b>')
        return
    
    node = AddTwoIntsClient()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    rclpy.spin_until_future_complete(node, future)
    
    try:
        result = future.result()
        node.get_logger().info(f'Result: {result.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Parameter Example

### Parameter Node (parameter_node.py)
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class ParameterNode(Node):
    """Node demonstrating parameter usage."""
    
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with descriptions
        self.declare_parameter(
            'my_string',
            'default_value',
            ParameterDescriptor(description='A string parameter')
        )
        self.declare_parameter(
            'my_int',
            42,
            ParameterDescriptor(description='An integer parameter')
        )
        self.declare_parameter(
            'my_double',
            3.14,
            ParameterDescriptor(description='A double parameter')
        )
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create timer to read parameters
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Parameter node started')

    def parameter_callback(self, params):
        """Called when parameters are changed."""
        for param in params:
            self.get_logger().info(
                f'Parameter {param.name} changed to: {param.value}'
            )
        return rclpy.parameter.SetParametersResult(successful=True)

    def timer_callback(self):
        """Periodically read and display parameters."""
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value
        my_double = self.get_parameter('my_double').value
        
        self.get_logger().info(
            f'Parameters: string={my_string}, int={my_int}, double={my_double}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Parameter YAML (params.yaml)
```yaml
/**:
  ros__parameters:
    my_string: "hello_ros2"
    my_int: 100
    my_double: 2.718
```

### Running with Parameters:
```bash
# Run with parameters from command line
ros2 run my_package parameter_node --ros-args -p my_string:="custom_value" -p my_int:=999

# Run with parameters from YAML file
ros2 run my_package parameter_node --ros-args --params-file params.yaml
```

---

## Using These Examples

### Setup:
1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Create a package:
```bash
ros2 pkg create --build-type ament_python my_examples --dependencies rclpy std_msgs example_interfaces
```

3. Copy example files to the package

4. Build:
```bash
cd ~/ros2_ws
colcon build --packages-select my_examples
source install/setup.bash
```

### Documentation:
Each example includes:
- Clear comments explaining the code
- Usage instructions
- Expected output
- Common pitfalls and solutions

### Learning Path:
1. Start with simple publisher-subscriber
2. Progress to services
3. Learn parameters
4. Move to actions and TF2
5. Finally, lifecycle and components

---

For more examples and tutorials, visit the [official ROS2 documentation](https://docs.ros.org/en/humble/Tutorials.html).
