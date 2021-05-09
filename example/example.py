#!/usr/bin/env python

from std_msgs.msg import Bool

from rosnodify import rosnode


@rosnode.publisher(Bool, '/output')
@rosnode.subscribe(Bool, '/test')
def sub(msg: Bool):
    rosnode.logger.info(f'{msg}')

    @rosnode.connection_based()
    def process():
        return {'/output': msg}


rosnode.register(node_name='test')
