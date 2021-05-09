#!/usr/bin/env python

from std_msgs.msg import Bool

from rosnodify import rosnode


@rosnode.parameters({'my_param': True})
@rosnode.parameters([('your_param', 1)])
@rosnode.publisher(Bool, '/output')
@rosnode.subscribe(Bool, '/test')
def sub(msg: Bool):
    rosnode.logger.info(f'{msg}')

    @rosnode.connection_based()
    def process():
        return {'/output': msg}


@rosnode.publisher(Bool, '/param')
@rosnode.timer(0.5)
def timer():
    msg = Bool(data=rosnode.get_parameter('my_param'))
    rosnode.publish(msg, '/param', False)


rosnode.register(node_name='test')
