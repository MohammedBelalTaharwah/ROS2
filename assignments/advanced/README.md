# Advanced ROS2 Assignments

This directory contains advanced-level assignments for mastering complex ROS2 features and real-world applications.

## Assignment List

1. [Assignment 1: Real-Time Performance and QoS](#assignment-1-real-time-performance-and-qos)
2. [Assignment 2: Multi-Robot Systems](#assignment-2-multi-robot-systems)
3. [Assignment 3: Component-Based Architecture](#assignment-3-component-based-architecture)
4. [Assignment 4: Security and DDS Configuration](#assignment-4-security-and-dds-configuration)
5. [Assignment 5: Integration Project](#assignment-5-integration-project)

---

## Assignment 1: Real-Time Performance and QoS

**Objective:** Optimize ROS2 system for real-time performance using advanced QoS settings.

### Tasks:
1. Implement real-time priority nodes
2. Configure DDS QoS policies for different scenarios
3. Measure and analyze latency
4. Optimize memory allocation
5. Implement deadline and lifespan QoS policies

### QoS Profiles to Implement:
- **Sensor Data:** Best effort, volatile
- **Control Commands:** Reliable, transient local
- **Critical Safety:** Reliable, keep last 1, deadline
- **Diagnostics:** Best effort, keep all

### Performance Requirements:
- Measure end-to-end latency
- Achieve <10ms response time for critical paths
- Handle 1000 Hz message rates
- Zero message loss for critical topics

### Code Example:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Define custom QoS
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    deadline=Duration(seconds=0.01)
)
```

### Deliverables:
- Implementation with different QoS profiles
- Performance benchmarks and graphs
- Analysis of QoS impact on system behavior
- Recommendations for real-world applications

---

## Assignment 2: Multi-Robot Systems

**Objective:** Design and implement a coordinated multi-robot system.

### Tasks:
1. Set up multiple robot instances with proper namespacing
2. Implement robot discovery mechanism
3. Create coordination algorithms
4. Handle inter-robot communication
5. Implement collision avoidance

### System Architecture:
```
Master Node (Coordinator)
    ├── Robot 1 (namespace: /robot1)
    │   ├── Navigation
    │   ├── Sensors
    │   └── Control
    ├── Robot 2 (namespace: /robot2)
    │   ├── Navigation
    │   ├── Sensors
    │   └── Control
    └── Robot N (namespace: /robotN)
```

### Requirements:
- Support dynamic robot addition/removal
- Centralized or distributed coordination
- Shared map/environment representation
- Task allocation and scheduling
- Communication failure handling

### Scenarios to Implement:
1. **Formation Control:** Robots maintain relative positions
2. **Task Allocation:** Distribute tasks among robots
3. **Collaborative Mapping:** Build joint map
4. **Resource Sharing:** Coordinate access to shared resources

### Testing:
- Simulate 3+ robots in Gazebo
- Test with different robot configurations
- Verify coordination under network delays
- Test failure recovery mechanisms

---

## Assignment 3: Component-Based Architecture

**Objective:** Build a modular system using ROS2 components and lifecycle nodes.

### Tasks:
1. Create managed lifecycle nodes
2. Implement state machine for node lifecycle
3. Use intra-process communication
4. Build composable components
5. Create a component container system

### Lifecycle States:
- Unconfigured
- Inactive
- Active
- Finalized

### Requirements:
- Implement lifecycle callbacks
- Handle state transitions properly
- Use composition for zero-copy communication
- Create reusable components
- Implement proper cleanup

### Component Examples:
```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

class SensorComponent(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Start operations
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Pause operations
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Cleanup resources
        return TransitionCallbackReturn.SUCCESS
```

### Deliverables:
- Multiple lifecycle components
- Component composition examples
- Performance comparison (intra-process vs inter-process)
- Documentation of lifecycle patterns

---

## Assignment 4: Security and DDS Configuration

**Objective:** Implement ROS2 security features and custom DDS configurations.

### Tasks:
1. Enable ROS2 security (SROS2)
2. Generate and manage security certificates
3. Configure DDS XML profiles
4. Implement access control policies
5. Set up encrypted communication

### Security Setup:
```bash
# Create keystore
ros2 security create_keystore demo_keystore

# Create keys for nodes
ros2 security create_key demo_keystore /talker
ros2 security create_key demo_keystore /listener

# Generate policy files
ros2 security create_permission demo_keystore /talker policies/talker.xml
```

### DDS Configuration:
- Custom discovery settings
- Transport configuration (UDP, TCP, Shared Memory)
- Domain participant QoS
- Network interface selection

### Requirements:
- Implement authentication and authorization
- Configure encrypted communication
- Test with different DDS vendors (Fast-DDS, CycloneDDS)
- Document security best practices

### Testing Scenarios:
1. Unauthorized node access prevention
2. Message encryption verification
3. Certificate expiration handling
4. Performance impact analysis

---

## Assignment 5: Integration Project

**Objective:** Build a complete robot application integrating all learned concepts.

### Project: Autonomous Warehouse Robot System

### System Components:
1. **Navigation Stack:**
   - SLAM or localization
   - Path planning
   - Obstacle avoidance
   - Nav2 integration

2. **Perception:**
   - Camera/LiDAR processing
   - Object detection
   - Environment mapping

3. **Task Management:**
   - Order processing
   - Route optimization
   - Fleet coordination

4. **Monitoring:**
   - Dashboard (web interface)
   - Health monitoring
   - Diagnostics

5. **Safety:**
   - Emergency stop
   - Collision detection
   - Battery monitoring

### Technical Requirements:
- Minimum 10 different nodes
- Use all major ROS2 communication patterns
- Implement proper error handling
- Real-time performance where needed
- Comprehensive logging and diagnostics
- Web-based monitoring interface

### Quality Requirements:
- Unit tests for components
- Integration tests for system
- Performance benchmarks
- Documentation (design, API, user guide)
- Deployment instructions

### Architecture Design:
```
┌─────────────────────────────────────────────┐
│           Task Management Layer              │
│  (Order Processing, Route Planning)          │
└────────────┬────────────────────────────────┘
             │
┌────────────┴────────────────────────────────┐
│          Navigation & Control Layer          │
│  (Path Planning, Obstacle Avoidance)         │
└────────────┬────────────────────────────────┘
             │
┌────────────┴────────────────────────────────┐
│          Perception & Sensing Layer          │
│  (Sensors, Object Detection, Mapping)        │
└─────────────────────────────────────────────┘
```

### Deliverables:
1. Complete source code
2. Architecture documentation
3. API documentation
4. User manual
5. Test suite
6. Demo video
7. Performance analysis report
8. Deployment guide

---

## General Guidelines

### Code Quality:
- Follow industry best practices
- Use design patterns appropriately
- Implement comprehensive error handling
- Write maintainable and scalable code

### Testing:
- Unit tests for individual components
- Integration tests for system
- Performance tests
- Stress tests
- Failure scenario tests

### Documentation:
- Architecture diagrams (UML, system diagrams)
- API documentation (Doxygen/Sphinx)
- User guides with examples
- Troubleshooting guides
- Performance tuning guides

### Version Control:
- Use Git with meaningful commits
- Feature branches for development
- Pull requests for review
- Semantic versioning

## Evaluation Criteria

1. **Technical Excellence (40%)**
   - Correct implementation of ROS2 features
   - Performance optimization
   - Security implementation
   - Error handling

2. **System Design (25%)**
   - Architecture quality
   - Modularity and reusability
   - Scalability considerations

3. **Code Quality (20%)**
   - Readability and maintainability
   - Best practices adherence
   - Testing coverage

4. **Documentation (15%)**
   - Completeness
   - Clarity
   - Professional presentation

## Additional Resources

- [ROS2 Real-Time](https://design.ros2.org/articles/realtime_background.html)
- [SROS2 Security](https://docs.ros.org/en/humble/Tutorials/Advanced/Security.html)
- [ROS2 Components](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [DDS Security](https://www.omg.org/spec/DDS-SECURITY/)

## Tips for Success

1. Start with clear requirements and design
2. Break complex problems into smaller tasks
3. Test frequently and incrementally
4. Profile and optimize performance
5. Document as you code
6. Seek feedback early and often

Good luck with your advanced assignments! 🚀🤖
