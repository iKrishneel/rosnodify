#!/usr/bin/env python


def assert_t(*, obj):
    def _wrapper(func):
        def _execute(args):
            assert isinstance(arg, obj), f'Excepted {type} but got {type(arg)}'
            return func(arg)

        return _execute

    return _wrapper


def exception_t(*, error):
    def _wrapper(func):
        def _execute(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except error as e:
                print(f'\n{error.__name__} {e} has been handled')

        return _execute

    return _wrapper


@exception_t(error=KeyError)
@assert_t(obj=str)
def main2(y):
    print("Val is: ", y)
    x = dict()
    print(x[y])


if __name__ == '__main__':
    main2("this")
