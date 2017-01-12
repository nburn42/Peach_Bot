# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from nav_msgs/MapMetaData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import genpy

class MapMetaData(genpy.Message):
  _md5sum = "10cfc8a2818024d3248802c00c95f11b"
  _type = "nav_msgs/MapMetaData"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['map_load_time','resolution','width','height','origin']
  _slot_types = ['time','float32','uint32','uint32','geometry_msgs/Pose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       map_load_time,resolution,width,height,origin

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MapMetaData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.map_load_time is None:
        self.map_load_time = genpy.Time()
      if self.resolution is None:
        self.resolution = 0.
      if self.width is None:
        self.width = 0
      if self.height is None:
        self.height = 0
      if self.origin is None:
        self.origin = geometry_msgs.msg.Pose()
    else:
      self.map_load_time = genpy.Time()
      self.resolution = 0.
      self.width = 0
      self.height = 0
      self.origin = geometry_msgs.msg.Pose()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2If2I7d().pack(_x.map_load_time.secs, _x.map_load_time.nsecs, _x.resolution, _x.width, _x.height, _x.origin.position.x, _x.origin.position.y, _x.origin.position.z, _x.origin.orientation.x, _x.origin.orientation.y, _x.origin.orientation.z, _x.origin.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.map_load_time is None:
        self.map_load_time = genpy.Time()
      if self.origin is None:
        self.origin = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 76
      (_x.map_load_time.secs, _x.map_load_time.nsecs, _x.resolution, _x.width, _x.height, _x.origin.position.x, _x.origin.position.y, _x.origin.position.z, _x.origin.orientation.x, _x.origin.orientation.y, _x.origin.orientation.z, _x.origin.orientation.w,) = _get_struct_2If2I7d().unpack(str[start:end])
      self.map_load_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2If2I7d().pack(_x.map_load_time.secs, _x.map_load_time.nsecs, _x.resolution, _x.width, _x.height, _x.origin.position.x, _x.origin.position.y, _x.origin.position.z, _x.origin.orientation.x, _x.origin.orientation.y, _x.origin.orientation.z, _x.origin.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.map_load_time is None:
        self.map_load_time = genpy.Time()
      if self.origin is None:
        self.origin = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 76
      (_x.map_load_time.secs, _x.map_load_time.nsecs, _x.resolution, _x.width, _x.height, _x.origin.position.x, _x.origin.position.y, _x.origin.position.z, _x.origin.orientation.x, _x.origin.orientation.y, _x.origin.orientation.z, _x.origin.orientation.w,) = _get_struct_2If2I7d().unpack(str[start:end])
      self.map_load_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2If2I7d = None
def _get_struct_2If2I7d():
    global _struct_2If2I7d
    if _struct_2If2I7d is None:
        _struct_2If2I7d = struct.Struct("<2If2I7d")
    return _struct_2If2I7d
