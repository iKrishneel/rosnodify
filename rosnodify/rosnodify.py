#!/usr/bin/env python

from dataclasses import dataclass, field
from threading import Lock
from enum import Enum, auto

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException

from message_filters import ApproximateTimeSynchronizer, Subscriber

from utils import exception_t, assert_t


class Type(Enum):
    SUB = auto()
    PUB = auto()
    CLIENT = auto()
    SERVER = auto()
    PARAM = auto()


@dataclass(init=True)
class Registry(object):
    is_lazy: bool = False
    parameters: dict = field(default_factory=dict)

    def __post_init__(self):
        self.parameters = dict()
        if self.is_lazy:
            self.sub_registry = []
            self.pub_registry = []
            self.client_registry = []
            self.srv_registry = []
        else:
            self.sub_registry = dict()
            self.pub_registry = dict()
            self.client_registry = dict()
            self.srv_registry = dict()

    def __call__(self, _type, **kwargs: dict):
        if _type == Type.SUB:
            _registry = self.sub_registry
        elif _type == Type.PUB:
            _registry = self.pub_registry
        elif _type == Type.CLIENT:
            _registry = self.client_registry
        else:
            raise TypeError(f'Unknown Type {_type}')

        if self.is_lazy:
            _registry.append(kwargs)
        else:
            key = kwargs['topic']
            assert key

            handle = kwargs['handle']
            # assert handle and key in _registry, f'{_registry}'
            _registry[key] = handle


lazy_registry = Registry(is_lazy=True)


@dataclass(init=True)
class Nodify(object):

    _has_init: bool = False

    def __call__(self, arg, **kwargs: dict):
        if callable(arg):
            self.create_node(arg(), **kwargs)
            return arg

        @assert_t(obj=str)
        def _register(arg):
            def _wrapper(func_or_cls):
                self.create_node(arg, **kwargs)
                func_or_cls()
                return func_or_cls

            return _wrapper

        return _register(arg)

    def create_node(self, node_name: str, **kwargs: dict):
        if self._has_init:
            raise Exception('The node was already initialized')

        assert isinstance(node_name, str)
        self._node = rclpy.create_node(node_name, **kwargs)
        self._registry = Registry(is_lazy=False)
        self._has_init = True

    def create_subscriber(self, **kwargs):
        try:
            self._node.create_subscription(**kwargs)
        except (TypeError, AttributeError) as e:
            print(e)

    @classmethod
    def subscribe(cls, msg_type, topic: str, qos: int = 1, **kwargs: dict):
        assert isinstance(topic, str) and callable(msg_type)

        def wrapper(func_or_cls):
            assert callable(func_or_cls)
            args = dict(
                msg_type=msg_type,
                topic=topic,
                callback=func_or_cls,
                qos_profile=qos,
                **kwargs,
            )
            lazy_registry(Type.SUB, **args)
            return func_or_cls

        return wrapper

    # @assert_t('topic', str)
    @classmethod
    def publisher(cls, msg_type, topic: str, qos: int = 1, **kwargs: dict):
        assert callable(msg_type)

        def wrapper(func_or_cls):
            assert callable(func_or_cls)
            args = dict(
                msg_type=msg_type, topic=topic, qos_profile=qos, **kwargs
            )
            lazy_registry(Type.PUB, **args)
            return func_or_cls

        return wrapper

    @classmethod
    def client(cls, srv_type, srv_name: str, **kwargs: dict):
        assert isinstance(srv_name, str) and callable(srv_type)
        timeout = kwargs.get('timeout', 5.0)

        def wrapper(func):
            assert callable(func)
            args = dict(
                srv_type=srv_type, srv_name=srv_name, timeout=timeout, **kwargs
            )
            lazy_registry.client_registry(Type.CLIENT, **args)
            return func

        return wrapper

    @classmethod
    def serivce(cls, srv_type, srv_name: str, **kwargs: dict):
        assert isinstance(srv_name, str) and callable(srv_type)
        timeout = kwargs.get('timeout', 5.0)

        def wrapper(func):
            assert callable(func)
            args = dict(
                srv_type=srv_type,
                srv_name=srv_name,
                callback=func,
                timeout=timeout,
                **kwargs,
            )
            lazy_registry.srv_registry(Type.SERVER, **args)
            return func

        return wrapper

    @classmethod
    def parameters(cls, params: list):
        assert isinstance(params, (list, tuple, dict))

        def wrapper(func):
            param_dict = params if isinstance(params, dict) else {}
            if isinstance(params, (list, tuple)):
                param_dict = {key: value for key, value in params}
            lazy_registry.parameters.update(param_dict)
            return func

        return wrapper

    @classmethod
    def timer(cls, timer_period: float):
        assert isinstance(timer_period, float)

        def wrapper(func):
            assert callable(func)
            # TODO: timer
            return func

        return wrapper

    def approx_time_sync(self, msg_topic_list: list, **kwargs: dict):
        assert isinstance(msg_topic_list, (tuple, list))

        def wrapper(func):
            assert callable(func)
            ApproximateTimeSynchronizer(
                [
                    Subscriber(self._node, *msg_topic)
                    for msg_topic in msg_topic_list
                ],
                **kwargs,
            ).registerCallback(func)
            return func

        return wrapper

    def connection_based(self):
        def wrapper(func):
            assert callable(func)
            msgs_dict = func()
            assert isinstance(msgs_dict, dict)
            for topic, msg in msgs_dict.items():
                self.publish(msg, topic, check_connection=True)
            return func

        return wrapper

    def publish(self, msg, topic: str, check_connection: bool = True):
        pub = self.get_publisher(topic=topic)
        if check_connection and pub.get_subscription_count() == 0:
            self.logger.info(f'Publish {topic} has no subscription')
            return
        pub.publish(msg)

    def _hook(self):
        for args in lazy_registry.sub_registry:
            params = dict(
                topic=args['topic'], handle=self.create_subscriber(**args)
            )
            self._registry(Type.SUB, **params)

        for args in lazy_registry.pub_registry:
            params = dict(
                topic=args['topic'], handle=self._node.create_publisher(**args)
            )
            self._registry(Type.PUB, **params)

        for args in lazy_registry.client_registry:
            timeout = args.pop('timeout')
            client = self._node.create_client(**args)
            while not client.wait_for_service(timeout_sec=timeout):
                self.logger.warn(
                    f'Service {srv_name} not available, waiting...'
                )
            params = dict(topic=args['topic'], handle=client)
            self._registry(Type.CLIENT, **params)

        for args in lazy_registry.srv_registry:
            timeout = args.pop('timeout')
            srv = self._node.create_service(**args)
            params = dict(topic=args['topic'], handle=srv)
            self._registry(Type.SERVER, **params)

    """
    def hook(self, *args, **kwargs):
        print(args, kwargs)
        def wrapper(func):
            self._hook()
            func()
            return func
        return wrapper
    """

    @exception_t(error=KeyboardInterrupt)
    def spin(self, once: bool = False):
        if once:
            rclpy.spin_once(self._node)
            return
        while rclpy.ok():
            rclpy.spin(self._node)

    def register(self, *, node_name: str = None, **kwargs):
        if self._has_init and node_name is not None:
            self.logger.warn(
                f'The node was already initialized by name {self._node.get_name()} '
                f'The given name {node_name} will be ignored'
            )

        if not self._has_init:
            if node_name is None:
                raise ValueError(
                    'Node was not initalized. Node name is required'
                )
            else:
                self.create_node(node_name)

        self._hook()

        once = kwargs.get('spin_once', False)
        self.spin(once=once)

    @exception_t(error=KeyError)
    @assert_t(obj=str)
    def get_subscriber(self, topic: str):
        return self._registry.sub_registry[topic]

    # @assert_t(obj=str)
    def get_publisher(self, topic: str):
        return self._registry.pub_registry[topic]

    @property
    def logger(self):
        return self._node.get_logger()


def node_init(args=None):
    rclpy.init(args=args)
    return Nodify()


from std_msgs.msg import Bool


rosnode = node_init()


@rosnode.publisher(Bool, '/output')
@rosnode.subscribe(Bool, '/test')
def sub(msg: Bool):
    rosnode.logger.info(f'{msg}')

    @rosnode.connection_based()
    def process():
        return {'/output': msg}

    # rosnode.publish(msg, '/output')


# @rosnode
def main():
    return "test"


rosnode.register(node_name='test')


import IPython

# IPython.embed()
