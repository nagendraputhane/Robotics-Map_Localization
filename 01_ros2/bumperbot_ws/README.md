# ROS 2 Publisher/Subscriber Example

---

## Overview

- **SimplePublisher**  
  This node publishes a string message to the topic `chatter` at 1 Hz. Every second, it sends a message like `"Hello ROS 2 - counter: X"`, where `X` increments with each published message.

- **SimpleSubscriber**  
  This node subscribes to the `chatter` topic and prints out the received messages. The callback function is set up to display the message contents in the console.

---

## Code Structure

### Publisher Node

- **File:** `simple_publisher.cpp`
- **Key points:**
  - Inherits from `rclcpp::Node`.
  - Initializes a publisher for `std_msgs::msg::String` messages on the topic `chatter`.
  - Uses a wall timer to call a callback function every 1 second.
  - Publishes a message with an incremented counter.

### Subscriber Node

- **File:** `simple_subscriber.cpp`
- **Key points:**
  - Inherits from `rclcpp::Node`.
  - Creates a subscription to the `chatter` topic.
  - Uses `std::bind` to connect the incoming messages to a callback function.
  - The callback receives messages as a constant reference, ensuring the data isn’t modified.

---
