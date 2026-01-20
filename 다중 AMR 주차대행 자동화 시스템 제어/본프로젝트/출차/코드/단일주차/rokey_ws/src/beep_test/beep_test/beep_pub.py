import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


class NodeBeep(Node):
    def __init__(self):
        super().__init__("node_beep")
        self.beep_pub = self.create_publisher(
            AudioNoteVector,
            '/robot1/cmd_audio',
            10
        )
        self.timer = self.create_timer(2.0, self.publish_beep)
        self.published = False
        self.get_logger().info("NodeBeep 시작")
    def publish_beep(self):
        if self.published:
            return
        msg = AudioNoteVector()
        msg.append = False
        notes = [
            (880, 0, 300000000),
            (440, 0, 300000000),
            (880, 0, 300000000),
            (440, 0, 300000000)
        ]
        for freq, sec, nsec in notes:
            n = AudioNote()
            n.frequency = freq
            n.max_runtime = Duration(sec=sec, nanosec=nsec)
            msg.notes.append(n)
        self.get_logger().info("비프음 발행")
        self.beep_pub.publish(msg)
        self.published = True
        
def main(args=None):
    rclpy.init(args=args)
    node = NodeBeep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()