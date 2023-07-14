#!/usr/bin/env python

from geometry_msgs.msg import PointStamped
from rosnodify import rosnode
from std_msgs.msg import Header


@rosnode.parameters({'my_param': True, 'time_diff': 1e-1})
@rosnode.parameters([('your_param', 1)])
@rosnode.publisher(PointStamped, '/output')
@rosnode.subscribe(PointStamped, '/param')
def sub(msg: PointStamped):
    @rosnode.connection_based
    @rosnode.filter_buffer(msg.header)
    def process():
        return {'/output': msg}


@rosnode.publisher(PointStamped, '/param')
@rosnode.timer(0.5)
def timer():
    param = rosnode.get_parameter('my_param')
    header = Header(frame_id='', stamp=rosnode.clock_now.to_msg())
    point = PointStamped(header=header)
    point.point.x = 0.1 if param else 0.5
    rosnode.publish(point, '/param')


rosnode.register(node_name='test')
