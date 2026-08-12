#!/usr/bin/env python3
"""
Trigger Input Node
------------------
Manual test script: reads a user command from the terminal (input()),
publishes it to the /vlm/trigger topic so vlm_client_node.py picks it up
and starts inference.
Run:
  ros2 run reasoning trigger_input_node
Type a command then Enter -> publishes immediately. Type 'q' or Ctrl+C to quit.
Note: this is ONLY a MANUAL TEST tool (a replacement for `ros2 topic pub`
by hand). It is not the final user interface - it may later be replaced by
voice-to-text, a web form, etc., all of which just need to publish a String
to the same topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class TriggerInputNode(Node):
    def __init__(self):
        super().__init__('trigger_input_node')
        self.declare_parameter("trigger_topic", "/vlm/trigger")
        trigger_topic = self.get_parameter("trigger_topic").value
        self.pub_ = self.create_publisher(String, trigger_topic, 10)
        self.get_logger().info(
            f"TriggerInputNode ready. Type a command then Enter to publish to "
            f"'{trigger_topic}'. Type 'q' to quit."
        )
    def publish_instruction(self, text: str):
        msg = String()
        msg.data = text
        self.pub_.publish(msg)
        self.get_logger().info(f"Published: '{text}'")
def main(args=None):
    rclpy.init(args=args)
    node = TriggerInputNode()
    try:
        while rclpy.ok():
            try:
                text = input("Enter a command (q to quit): ").strip()
            except EOFError:
                break
            if text.lower() == 'q':
                break
            if not text:
                continue
            node.publish_instruction(text)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
