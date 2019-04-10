import struct
import sys

__author__ = 'paoolo'


class Socket(object):
    def __init__(self, socket):
        self.__socket = socket
        self.__checksum = 0

    def close(self):
        self.__socket.close()

    def get_checksum(self):
        return self.__checksum

    def read(self, size):
        remaining = size
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        if sys.version_info >= (3,0,0):
            buf = str(buf, 'UTF-8')
        else:
            buf = str(buf)
        return buf

    def write(self, char):
        if sys.version_info >= (3,0,0):
            char = bytes(char, 'UTF-8')
        self.__socket.send(char)

    def send_command(self, address, command):
        self.__checksum = address
        self.__socket.send(chr(address))
        self.__checksum += command
        self.__socket.send(chr(command))

    def read_byte(self):
        remaining = 1
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>B', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        remaining = 1
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>b', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_word(self):
        remaining = 2
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>H', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        remaining = 2
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>h', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_long(self):
        remaining = 4
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>L', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        remaining = 4
        buf = bytearray(remaining)
        view = memoryview(buf)
        while remaining > 0:
            nbytes = self.__socket.recv_into(view, remaining)
            view = view[nbytes:]
            remaining -= nbytes
        res = buf
        if len(res) > 0:
            val = struct.unpack('>l', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum += val & 0xFF
        return self.__socket.send(struct.pack('>B', val))

    def write_sbyte(self, val):
        self.__checksum += val & 0xFF
        return self.__socket.send(struct.pack('>b', val))

    def write_word(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__socket.send(struct.pack('>H', val))

    def write_sword(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__socket.send(struct.pack('>h', val))

    def write_long(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__socket.send(struct.pack('>L', val))

    def write_slong(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__socket.send(struct.pack('>l', val))
