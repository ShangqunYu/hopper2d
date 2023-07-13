"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class terrain_lcmt(object):
    __slots__ = ["floor"]

    __typenames__ = ["double"]

    __dimensions__ = [[1000]]

    def __init__(self):
        self.floor = [ 0.0 for dim0 in range(1000) ]

    def encode(self):
        buf = BytesIO()
        buf.write(terrain_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>1000d', *self.floor[:1000]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != terrain_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return terrain_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = terrain_lcmt()
        self.floor = struct.unpack('>1000d', buf.read(8000))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if terrain_lcmt in parents: return 0
        tmphash = (0x73b4665c25b4a6c6) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if terrain_lcmt._packed_fingerprint is None:
            terrain_lcmt._packed_fingerprint = struct.pack(">Q", terrain_lcmt._get_hash_recursive([]))
        return terrain_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", terrain_lcmt._get_packed_fingerprint())[0]

