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

@rosnode.client(AddTwoInts, srv_name)
@rosnode.service(AddTwoInts, srv_name)
def service(request, response):
    response.sum = request.a + request.b
    return response


@rosnode.publisher(PointStamped, '/param')
@rosnode.timer(0.5)
def timer():
    a, b = 1, 2
    request = AddTwoInts.Request(a=a, b=b)
    result = rosnode.call_async(request, srv_name, True)
    
    rosnode.logger.warn(f"Sum of {a} + {b} = {result.result()}")


rosnode.register(node_name='test_srv')
