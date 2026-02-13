import rclpy
from std_msgs.msg import Bool, Float32
from rclpy.node import Node
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node("auto_depth_test_node")

    depth_pub = node.create_publisher(Float32, "/depth", 10)
    auto_depth_activated_pub = node.create_publisher(Bool, "/auto_depth_activated", 10)

    for depth in [25.3, 25.29, 25.31]:
        depth_msg = Float32()
        depth_msg.data = depth
        depth_pub.publish(depth_msg) ##check auto depth node receives and adds to history

    time.sleep(2)

    activate_msg = Bool()
    activate_msg.data = True
    auto_depth_activated_pub.publish(activate_msg) ##activate auto depth, check target depth is set to arounnd 25.3

    time.sleep(2)

    activate_msg.data = True
    auto_depth_activated_pub.publish(activate_msg) ##should do nothing

    time.sleep(2)
    ## auto depth node should publish negative velocity
    for depth in [26.2, 26.4, 27.3]:
        depth_msg = Float32()
        depth_msg.data = depth
        depth_pub.publish(depth_msg)

    time.sleep(2)

    ## should publish positive velocity
    for depth in [24.2, 24.4, 24.3]:
        depth_msg = Float32()
        depth_msg.data = depth
        depth_pub.publish(depth_msg)

    time.sleep(2)

    ## should publish 0 velocity
    for depth in [25.3, 25.29, 25.28]:
        depth_msg = Float32()
        depth_msg.data = depth
        depth_pub.publish(depth_msg)

    time.sleep(2)

    activate_msg.data = False
    auto_depth_activated_pub.publish(activate_msg) ##deactive auto depth

    time.sleep(2)

    ## auto depth off so shouldn't publish velocity commands
    for depth in [25.3, 25.29, 25.28]:
        depth_msg = Float32()
        depth_msg.data = depth
        depth_pub.publish(depth_msg)



if __name__ == "__main__":
    main()