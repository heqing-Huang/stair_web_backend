import logging
import warnings
from dataclasses import dataclass, MISSING, Field
from functools import wraps
from typing import Dict, Callable, Any
from collections.abc import Iterable

_LOGGER = logging.getLogger(__name__)


class ExceptionNoMoreTypeCouldConverter(Exception):
    """
    不支持的类型转换
    """

    pass


class ConverterField(Field):
    def __init__(self, converter=None, *args, **kwargs):
        super(ConverterField, self).__init__(*args, **kwargs)
        self.converter = converter


def field_with_converter(
    *,
    converter=None,
    default: Any = MISSING,
    default_factory: Callable = MISSING,
    init=True,
    repr=True,
    hash=None,
    compare=True,
    metadata=None,
):
    if default is not MISSING and default_factory is not MISSING:
        raise ValueError("cannot specify both default and default_factory")

    return ConverterField(
        converter, default, default_factory, init, repr, hash, compare, metadata
    )


def post_init(obj):
    """
    进行强制类型转换

    如果对应的value 为None,则从field 中寻找default 和 default_factory 替换 None 值

    :param obj:
    :return:
    """
    cls_fields: Dict = getattr(obj, "__dataclass_fields__")
    for _key, _field in cls_fields.items():
        if isinstance(_field, ConverterField):
            _field: ConverterField
            if _field.converter is not None:
                _value_in = getattr(obj, _key)
                if _value_in is None:
                    if _field.default_factory is not MISSING:
                        _value_in = _field.default_factory()
                    elif _field.default is not MISSING:
                        _value_in = _field.default
                else:  # 仅当不为None 时,进行强制类型转换,我们认为用户默认值是能满足设定的
                    setattr(obj, _key, _field.converter(_value_in))

    post_converter = getattr(obj, "__post_converter__", None)
    if post_converter is not None and callable(post_converter):
        post_converter()


def convert_dataclass(cls, data):
    if cls is None:
        return None

    if isinstance(data, cls):
        return data
    elif isinstance(data, list):
        return cls(*data)
    elif isinstance(data, dict):
        return cls(**data)
    else:
        raise ExceptionNoMoreTypeCouldConverter(f"{cls}:{data}")


def iter_convert(data: Iterable, /, func: Callable):
    """
    可迭代对象的转换,用于偏函数填充的值放在后面
    :param data:
    :param func:
    :return:
    """
    if isinstance(data, Iterable):
        return list(map(func, data))
    else:
        raise ExceptionNoMoreTypeCouldConverter()


def dataclass_with_converter(
    cls=None,
    convert=None,
    /,
    *,
    init=True,
    repr=True,
    eq=True,
    order=False,
    unsafe_hash=False,
    frozen=False,
):
    """
    对dataclasses 中 dataclass 装饰器的封装

    会默认生成__post_init__ 函数,在初始化后执行强制类型转换 —— 可以不在Field 中申明强制类型转换,则不会强制类型转换

    会默认未创建的类，生成一个converter 类函数,可以提供默认的,将数据转换成类示例的方式 -—— 可以修改改类强制类型转换的函数

    :param cls:
    :param convert:额外的,用于强制类型转换的部分
    :param init:
    :param repr:
    :param eq:
    :param order:
    :param unsafe_hash:
    :param frozen:
    :return:
    """

    @wraps(dataclass)
    def in_call(cls):
        setattr(cls, "__post_init__", post_init)
        cls_in = dataclass(
            init=init,
            repr=repr,
            eq=eq,
            order=order,
            unsafe_hash=unsafe_hash,
            frozen=frozen,
        )(cls)

        if convert is None:
            convert_func = convert_dataclass
        else:
            convert_func = convert

        convert_class_method = classmethod(convert_func)
        setattr(cls_in, "converter", convert_class_method)
        return cls_in

    if cls is None:
        return in_call

    return in_call(cls)
