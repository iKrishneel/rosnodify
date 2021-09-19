#!/usr/bin/env python

from datetime import datetime
from std_msgs.msg import String

from rosnodify import rosnode


@rosnode.parameters({'my_param': True})
@rosnode.parameters([('your_param', 1)])
@rosnode.publisher(String, '/output')
@rosnode.subscribe(String, '/param')
def sub(msg: String):
    rosnode.logger.info(f'{msg}')

    @rosnode.connection_based
    def process():
        return {'/output': msg}


@rosnode.publisher(String, '/param')
@rosnode.timer(0.5)
def timer():
    now = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
    param = rosnode.get_parameter('my_param')
    msg = f'{now}: my_param: {param}'
    rosnode.publish(String(data=msg), '/param', False)


rosnode.register(node_name='test')
