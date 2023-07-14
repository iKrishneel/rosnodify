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


@rosnode.publisher(PointStamped, '/param')
@rosnode.service(AddTwoInts, srv_name)
def service(request, response):
    rosnode.logger.info("Got service call")
    response.sum = request.a + request.b
    rosnode.logger.info(f"Responding with {response}")
    return response




rosnode.register(node_name='test_srv')
