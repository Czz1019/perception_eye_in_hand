"""Read only Marker.points length; avoids constructing tens of thousands of Python points."""
import struct

def marker_points_count(raw):
    # ROS 2 CDR encapsulation: all alignment below is relative to its 4-byte header.
    b=memoryview(raw)[4:]
    if bytes(raw[:2]) not in (b'\x00\x01',b'\x00\x00'):
        raise ValueError('Unsupported CDR encapsulation')
    endian='<' if raw[1]==1 else '>'
    offset=8 # Header stamp: sec and nanosec
    for _ in range(2): # Header.frame_id, then Marker.ns
        offset=(offset+3)&~3
        n=struct.unpack_from(endian+'I',b,offset)[0]
        offset+=4+n
    offset=(offset+3)&~3
    offset+=12 # id, type, action
    offset=(offset+7)&~7
    offset+=56+24+16+8+1 # pose, scale, color, lifetime, frame_locked
    offset=(offset+3)&~3
    return struct.unpack_from(endian+'I',b,offset)[0]
