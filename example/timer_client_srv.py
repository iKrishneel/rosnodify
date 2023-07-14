#!/usr/bin/env python

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

try:
    from example_interfaces.srv import AddTwoInts
except ImportError as e:
    print(
        '\033[33mexample_interface is not installed.',
        'Fist install it apt-get install ros-${ROS_DISTRO}-example-interfaces. \033[0m'
    )
    import sys

    sys.exit()

from rosnodify import rosnode


srv_name = '/two_ints'

@rosnode.client(AddTwoInts, srv_name, timeout=1.0)


@rosnode.publisher(PointStamped, '/param')
@rosnode.timer(0.5)
def timer():
    a, b = 1, 2
    rosnode.logger.info("Sending request")
    request = AddTwoInts.Request(a=a, b=b)
    rosnode.future = rosnode.call_async(request, srv_name, wait=False)


@rosnode.main
def main():
    pass


rosnode.register(node_name='test_srv', spin_once=True)

import rclpy
while rclpy.ok():
   rclpy.spin_once(rosnode._node)
   future = getattr(rosnode, "future", None)
   if future.done():
     result = future.result()
     rosnode.logger.warn(f"Result {result}")
