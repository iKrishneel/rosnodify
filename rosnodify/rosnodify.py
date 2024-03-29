#!/usr/bin/env python

from dataclasses import dataclass, field
from threading import Lock
from enum import Enum, auto

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException

from message_filters import ApproximateTimeSynchronizer, Subscriber

from .utils import exception_t, assert_t


__all__ = [
    'Nodify',
]


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
    timers: list = field(default_factory=list)

    def __post_init__(self):
        self.parameters = dict()
        self.timers = []

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
        elif _type == Type.SERVER:
            _registry = self.srv_registry
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
    _main_func = None

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

    def __getattr__(self, attr):
        return getattr(self._node, attr)

    def create_node(self, node_name: str, **kwargs: dict):
        if self._has_init:
            raise Exception('The node was already initialized')

        assert isinstance(node_name, str)
        self._node = rclpy.create_node(node_name, **kwargs)
        self._registry = Registry(is_lazy=False)
        self._has_init = True

    def create_subscriber(self, **kwargs):
        try:
            return self._node.create_subscription(**kwargs)
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
        timeout = kwargs.pop('timeout', 5.0)

        def wrapper(func):
            assert callable(func)
            args = dict(
                srv_type=srv_type, srv_name=srv_name, timeout=timeout, **kwargs
            )
            lazy_registry(Type.CLIENT, **args)
            return func

        return wrapper

    @classmethod
    def service(cls, srv_type, srv_name: str, **kwargs: dict):
        assert isinstance(srv_name, str) and callable(srv_type)

        def wrapper(func):
            assert callable(func)
            args = dict(
                srv_type=srv_type,
                srv_name=srv_name,
                callback=func,
                **kwargs,
            )
            lazy_registry(Type.SERVER, **args)
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
            lazy_registry.timers.append([timer_period, func])
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

    def connection_based(self, func_or_cls=None):
        def wrapper(func):
            assert callable(func)
            msgs_dict = func()
            if isinstance(msgs_dict, dict):
                for topic, msg in msgs_dict.items():
                    self.publish(msg, topic, check_connection=True)
            return func

        return wrapper(func_or_cls) if func_or_cls else wrapper

    def publish(self, msg, topic: str, check_connection: bool = True):
        pub = self.get_publisher(topic=topic)
        if check_connection and pub.get_subscription_count() == 0:
            self.logger.info(
                f'Publish {topic} has no subscription', throttle_duration_sec=5
            )
            return
        pub.publish(msg)

    def filter_buffer(self, header, **kwargs):
        def wrapper(func):
            return func if self._is_msg_time_valid(header, **kwargs) else None

        return wrapper

    def timeit(self, func, once: bool = True, throttle: int = None):

        def _wrapper(*args, **kwargs):
            start = self.clock_now
            result = func(*args, **kwargs)
            ptime = (self.clock_now - start).nanoseconds / 1E9
            self.logger.info(
                f'Processing time: {ptime}',
                once=once,
                throttle_duration_sec=throttle,
            )
            return result

        return _wrapper

    def _is_msg_time_valid(self, header, time_diff: float = None):
        time_diff = time_diff if time_diff else self.get_parameter('time_diff')
        if time_diff < 0.0:
            return True

        try:
            bridge_time = rclpy.duration.Duration(seconds=0.0)
            now = self.clock_now - bridge_time
            msg_time = rclpy.time.Time.from_msg(header.stamp)
            duration = rclpy.duration.Duration(seconds=time_diff)

            if (now - msg_time) > duration:
                d = (now - msg_time).nanoseconds / 1E9
                c = duration.nanoseconds / 1E9
                raise ValueError(
                    f'\nObselete message: {d} and diff should be: {c}'
                )
        except ValueError as e:
            self.logger.warn(f'{e}', throttle_duration_sec=max(time_diff, 2.0))
            return False
        except AttributeError as e:
            self.logger.info(f'{e}', once=True)
        return True

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

        for args in lazy_registry.srv_registry:
            srv = self._node.create_service(**args)
            params = dict(topic=args['srv_name'], handle=srv)
            self._registry(Type.SERVER, **params)

        for args in lazy_registry.client_registry:
            timeout = args.pop('timeout')
            client = self._node.create_client(**args)
            while not client.wait_for_service(timeout_sec=timeout):
                srv_name = args['srv_name']
                self.logger.warn(
                    f'Service {srv_name} not available, waiting...'
                )
            params = dict(topic=args['srv_name'], handle=client)
            self._registry(Type.CLIENT, **params)

        for args in lazy_registry.timers:
            timer = self._node.create_timer(*args)
            self._registry.timers.append(timer)

        for param_name, param_value in lazy_registry.parameters.items():
            self.create_param(param_name, param_value)

    @exception_t(error=ParameterAlreadyDeclaredException)
    def create_param(self, param_name: str, param_value):
        assert self._node.declare_parameter(param_name, param_value)

    def get_parameter(self, param_name: str):
        param = self._node.get_parameter(param_name).get_parameter_value()
        ptype = list(param.get_fields_and_field_types().keys())[param.type]
        return getattr(param, ptype)

    def get_client(self, srv_name: str):
        assert srv_name in self._registry.client_registry, f'{srv_name} not yet registed'
        return self._registry.client_registry[srv_name]

    def call_async(self, request, srv_name: str, wait: bool = False, timeout: float = None):
        client = self.get_client(srv_name)
        future = client.call_async(request)
        if wait:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            return future.result()
        return future

    @exception_t(error=KeyboardInterrupt)
    def spin(self, once: bool = False):
        if once:
            rclpy.spin_once(self._node)
            return
        while rclpy.ok():
            rclpy.spin(self._node)

    def main(self, args=None):
        def wrapper(func):
            self._main_func = func
            return func

        if callable(args):
            return wrapper(args)
        else:
            return wrapper

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

        if self._main_func:
            self._main_func()

        do_spin = kwargs.get('do_spin', True)
        once = kwargs.get('spin_once', False)
        if do_spin:
            self.spin(once=once)

    # @exception_t(error=KeyError)
    # @assert_t(obj=str)
    def get_subscriber(self, topic: str):
        return self._registry.sub_registry[topic]

    # @assert_t(obj=str)
    def get_publisher(self, topic: str):
        return self._registry.pub_registry[topic]

    @property
    def logger(self):
        return self._node.get_logger()

    @property
    def clock_now(self):
        return self._node.get_clock().now()


def node_init(args=None):
    rclpy.init(args=args)
    return Nodify()
