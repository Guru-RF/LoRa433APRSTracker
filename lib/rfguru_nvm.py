try:
    from typing import Union
except ImportError:
    pass

import struct
from io import BytesIO

import microcontroller
import msgpack


class NonVolatileMemory:
    def __init__(self, debug=False):
        self.debug = debug

    def save_data(self, data: Union[object, list, dict, int, float, str]) -> None:
        packed_data_io = BytesIO()
        msgpack.pack(data, packed_data_io)
        packed_data_io.seek(0)
        len_of_data = len(packed_data_io.read())
        packed_data_io.seek(0)
        size_packed = struct.pack("i", len_of_data)
        bytes_io_out = BytesIO()
        bytes_io_out.write(size_packed)
        bytes_io_out.write(packed_data_io.read())
        bytes_io_out.seek(0)
        total_size = len_of_data + 4
        bytes_to_save = bytes_io_out.read()
        microcontroller.nvm[0:total_size] = bytes_to_save

    def read_data(self) -> Union[object, list, dict, int, float, str]:
        size_unpacked = struct.unpack("i", microcontroller.nvm[:4])[0]
        _read_data = microcontroller.nvm[: size_unpacked + 4]
        b = BytesIO()
        b.write(_read_data[4 : size_unpacked + 4])
        b.seek(0)
        unpacked_data = msgpack.unpack(b)
        return unpacked_data
