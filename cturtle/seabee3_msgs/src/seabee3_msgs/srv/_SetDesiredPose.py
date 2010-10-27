"""autogenerated by genmsg_py from SetDesiredPoseRequest.msg. Do not edit."""
import roslib.message
import struct

import seabee3_msgs.msg
import geometry_msgs.msg

class SetDesiredPoseRequest(roslib.message.Message):
  _md5sum = "4ceeb5c5a4666b53df90ee0a681e0ac8"
  _type = "seabee3_msgs/SetDesiredPoseRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """seabee3_msgs/Vector3Masked pos
seabee3_msgs/Vector3Masked ori

================================================================================
MSG: seabee3_msgs/Vector3Masked
geometry_msgs/Vector3 mask
geometry_msgs/Vector3 mode
geometry_msgs/Vector3 values

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['pos','ori']
  _slot_types = ['seabee3_msgs/Vector3Masked','seabee3_msgs/Vector3Masked']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       pos,ori
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SetDesiredPoseRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pos is None:
        self.pos = seabee3_msgs.msg.Vector3Masked()
      if self.ori is None:
        self.ori = seabee3_msgs.msg.Vector3Masked()
    else:
      self.pos = seabee3_msgs.msg.Vector3Masked()
      self.ori = seabee3_msgs.msg.Vector3Masked()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_18d.pack(_x.pos.mask.x, _x.pos.mask.y, _x.pos.mask.z, _x.pos.mode.x, _x.pos.mode.y, _x.pos.mode.z, _x.pos.values.x, _x.pos.values.y, _x.pos.values.z, _x.ori.mask.x, _x.ori.mask.y, _x.ori.mask.z, _x.ori.mode.x, _x.ori.mode.y, _x.ori.mode.z, _x.ori.values.x, _x.ori.values.y, _x.ori.values.z))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.pos is None:
        self.pos = seabee3_msgs.msg.Vector3Masked()
      if self.ori is None:
        self.ori = seabee3_msgs.msg.Vector3Masked()
      end = 0
      _x = self
      start = end
      end += 144
      (_x.pos.mask.x, _x.pos.mask.y, _x.pos.mask.z, _x.pos.mode.x, _x.pos.mode.y, _x.pos.mode.z, _x.pos.values.x, _x.pos.values.y, _x.pos.values.z, _x.ori.mask.x, _x.ori.mask.y, _x.ori.mask.z, _x.ori.mode.x, _x.ori.mode.y, _x.ori.mode.z, _x.ori.values.x, _x.ori.values.y, _x.ori.values.z,) = _struct_18d.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_18d.pack(_x.pos.mask.x, _x.pos.mask.y, _x.pos.mask.z, _x.pos.mode.x, _x.pos.mode.y, _x.pos.mode.z, _x.pos.values.x, _x.pos.values.y, _x.pos.values.z, _x.ori.mask.x, _x.ori.mask.y, _x.ori.mask.z, _x.ori.mode.x, _x.ori.mode.y, _x.ori.mode.z, _x.ori.values.x, _x.ori.values.y, _x.ori.values.z))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.pos is None:
        self.pos = seabee3_msgs.msg.Vector3Masked()
      if self.ori is None:
        self.ori = seabee3_msgs.msg.Vector3Masked()
      end = 0
      _x = self
      start = end
      end += 144
      (_x.pos.mask.x, _x.pos.mask.y, _x.pos.mask.z, _x.pos.mode.x, _x.pos.mode.y, _x.pos.mode.z, _x.pos.values.x, _x.pos.values.y, _x.pos.values.z, _x.ori.mask.x, _x.ori.mask.y, _x.ori.mask.z, _x.ori.mode.x, _x.ori.mode.y, _x.ori.mode.z, _x.ori.values.x, _x.ori.values.y, _x.ori.values.z,) = _struct_18d.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_18d = struct.Struct("<18d")
"""autogenerated by genmsg_py from SetDesiredPoseResponse.msg. Do not edit."""
import roslib.message
import struct


class SetDesiredPoseResponse(roslib.message.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "seabee3_msgs/SetDesiredPoseResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SetDesiredPoseResponse, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      pass
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      pass
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
class SetDesiredPose(roslib.message.ServiceDefinition):
  _type          = 'seabee3_msgs/SetDesiredPose'
  _md5sum = '4ceeb5c5a4666b53df90ee0a681e0ac8'
  _request_class  = SetDesiredPoseRequest
  _response_class = SetDesiredPoseResponse
