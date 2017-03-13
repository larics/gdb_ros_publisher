import rospy
from geometry_msgs import msg as geometry_msgs
import tf2_ros

class GdbPublisher(object):
  def __init__(self, topic, msgtype, queue_size = 1):
    self.topic = topic
    self.msgtype = msgtype
    self.publisher = rospy.Publisher(topic, msgtype, queue_size=queue_size)

  def publish(self, serialized_message):
    message = self.msgtype()
    message.deserialize(serialized_message)
    self.publisher.publish(message)

class GdbPublisherDictionary(dict):
  def __init__(self, publishers = None):
    super(GdbPublisherDictionary, self).__init__()
    if(publishers != None):
      self.insert(publishers)

  def insert(self, publishers):
    try:
      iterator = iter(publishers)
      for publisher in publishers:
        self[publisher.topic] = publisher
    except TypeError:
      # add the single publisher that we received
      self[publishers.topic] = publishers

  def publish(self, topic, serialized_message):
    publisher = self.get(topic, None)

    if publisher != None:
      publisher.publish(serialized_message)
      return True

    return False

class GdbRosBreakpoint(gdb.Breakpoint):
  def __init__(self, location):
    super(GdbRosBreakpoint, self).__init__(location, internal = True)
    self.silent = True

  def stop(self):
    raise NotImplementedError()

class GdbMessageBreakpoint(GdbRosBreakpoint):
  # the default context extractors for universal breakpoint in ros/publish.h
  default_message_extractor = lambda self: gdb.selected_frame().read_var('message')
  default_serialized_message_extractor = lambda self: gdb.selected_frame().read_var('m')
  default_topic_extractor = lambda self: str(gdb.selected_frame().read_var('this').dereference()\
    ['impl_']['px'].dereference()['topic_']['_M_dataplus']['_M_p'].string())

  def __init__(self, location, context_extractor = lambda: {}):
    super(GdbMessageBreakpoint, self).__init__(location)
    self.context_extractor = context_extractor

  def stop(self):
    if(not hasattr(self, 'enclosing_node')):
      raise Exception('Missing node reference')

    context = self.context_extractor()
    if 'message_variable' not in context:
      context['message_variable'] = self.default_message_extractor()
    if 'serialized_variable' not in context:
      context['serialized_variable'] = self.default_serialized_message_extractor()
    if 'topic' not in context:
      context['topic'] = self.default_topic_extractor()

    self.enclosing_node.handle_message(**context)
    return False

class GdbTransformBreakpoint(GdbRosBreakpoint):
  default_transform_extractor = lambda: gdb.selected_frame().read_var('stamped_transform')

  def __init__(self, location, transform_extractor = default_transform_extractor, static_transform = False):
    super(GdbTransformBreakpoint, self).__init__(location)
    self.static_transform = static_transform
    self.transform_extractor = transform_extractor

  def stop(self):
    if(not hasattr(self, 'enclosing_node')):
      raise Exception('Missing node reference')

    self.enclosing_node.handle_transform(self.transform_extractor(), self.static_transform)
    return False

class GdbPublisherNode(object):
  def __init__ (self, publisher_dictionary, breakpoints, log_publisher = False, log_publisher_state = False):
    self.ros_handle = rospy.init_node('gdb_publisher')
    self.publisher_dictionary = publisher_dictionary
    self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    self.breakpoints = []
    self.add_breakpoints(breakpoints)
    self.log_publisher = log_publisher
    self.log_publisher_state = log_publisher_state
    self.enabled_states_saved = False

    # prepare some stuff for handling transforms
    try:
      self.transform_serialization_length_function = gdb.parse_and_eval(
        'ros::serialization::serializationLength<geometry_msgs::TransformStamped_<std::allocator<void> > >')
      self.char_pointer_type = gdb.lookup_type('unsigned char').pointer()
      self.malloc_function = gdb.parse_and_eval('malloc')
      self.free_function = gdb.parse_and_eval('free')
      self.sizeof_ostream = gdb.parse_and_eval('sizeof(ros::serialization::OStream)')
      self.ostream_pointer_type = gdb.lookup_type('ros::serialization::OStream').pointer()
      self.ostream_constructor = gdb.parse_and_eval('ros::serialization::OStream::OStream')
      # broken in gdb, but luckily we don't actually need it
      #self.ostream_destructor = gdb.parse_and_eval('ros::serialization::OStream::~OStream')
      self.transform_serialization_function = gdb.parse_and_eval(
        'ros::serialization::serialize<geometry_msgs::TransformStamped_<std::allocator<void> >,'
        'ros::serialization::OStream>(ros::serialization::OStream&, geometry_msgs::TransformStamped_<std::allocator<void> >)')
    except:
      raise ValueError("Could not prepare symbols. Has the executable file been loaded?")

    print "Initialized ROS publisher."

  def add_breakpoint(self, breakpoint):
    breakpoint.enclosing_node = self
    self.breakpoints.append(breakpoint)

  def add_breakpoints(self, breakpoints):
    try:
      iterator = iter(breakpoints)
      for breakpoint in breakpoints:
        self.add_breakpoint(breakpoint)
    except TypeError:
        self.add_breakpoint(breakpoint)

  def handle_message(self, message_variable, serialized_variable, topic):

    if topic[0] == '/':
      topic = topic[1:]

    if topic not in self.publisher_dictionary:
      if self.log_publisher:
        print "didn't publish message in {}".format(topic)
      # don't bother with executing the rest since we are not publishing this message
      return

    # discard the reference
    #if message_variable.type.code == gdb.TYPE_CODE_REF:
    #  message_variable = message_variable.referenced_value()
    # discard the const qualifier
    #message_type = message_variable.type.strip_typedefs().unqualified()

    # the above can be both done this way
    message_type = str(gdb.types.get_basic_type(message_variable.type))

    if(serialized_variable != None):
      # try to use the already serialized variable
      try:
        message_start = serialized_variable['message_start']
        # passed variable was unitialized and is a null pointer
        if message_start == 0:
          raise ValueError
      except ValueError:
        serialized_variable = None

    if(serialized_variable == None):
      # TODO do this only once, look up the result after
      serialize_function = gdb.parse_and_eval(
        'ros::serialization::serializeMessage< {} >'.format(message_type))
      serialized_variable = serialize_function(message_variable)
      message_start = serialized_variable['message_start']
      # TODO replace with malloc, serialize(), free

    # no need to use this since we can figure out the length as described below
    #serialization_length_function = gdb.parse_and_eval(
    #  'ros::serialization::serializationLength< {} >'.format(message_type))
    #serialized_length = int(serialization_length_function(message_variable))

    # in ros/serialization.h, serializeMessage(), num_bytes = len + 4
    serialized_length = serialized_variable['num_bytes'] - 4
    serialized_message = self.convert_serialized_to_python(message_start, serialized_length)

    self.publisher_dictionary.publish(topic, serialized_message)
    if self.log_publisher:
      print 'published {}, {}, {}'.format(topic, message_type, serialized_length)

  def handle_transform(self, transform_variable, static_transform):

    serialized_length = int(self.transform_serialization_length_function(transform_variable))

    serialization_buffer = self.malloc_function(serialized_length + 4).cast(self.char_pointer_type)
    serialization_ostream = self.malloc_function(self.sizeof_ostream).cast(self.ostream_pointer_type)
    self.ostream_constructor(serialization_ostream, serialization_buffer, serialized_length + 4)
    self.transform_serialization_function(serialization_ostream.dereference(), transform_variable)

    transform_serialized = self.convert_serialized_to_python(serialization_buffer, serialized_length)
    transform = geometry_msgs.TransformStamped()
    transform.deserialize(transform_serialized)

    if static_transform:
      self.static_tf_broadcaster.sendTransform(transform)
    else:
      self.tf_broadcaster.sendTransform(transform)

    # broken in gdb, but not important anyway
    #self.ostream_destructor(serialization_ostream, 1)
    self.free_function(serialization_ostream)
    self.free_function(serialization_buffer)

    if self.log_publisher:
      print 'published transform {} -> {}'.format(transform.header.frame_id, transform.child_frame_id)

  def enable_breakpoints(self, print_notice = True):
    for b in self.breakpoints:
      b.enabled = True
    if print_notice:
      print "Enabled ROS publisher."

  def disable_breakpoints(self, print_notice = True):
    for b in self.breakpoints:
      b.enabled = False
    if print_notice:
      print "Disabled ROS publisher."

  def save_state_and_disable_ros_publisher(self):
    if self.enabled_states_saved == True:
      raise Exception('This should not have happened, ROS publisher state has already been saved')
    self.breakpoint_enabled_states = [ b.enabled for b in self.breakpoints ]
    self.enabled_states_saved = True
    self.disable_breakpoints(False)
    if self.breakpoint_enabled_states[0] and self.log_publisher_state:
      print "Temporarily disabled ROS publisher."

  def restore_ros_publisher_state(self):
    if self.enabled_states_saved == True:
      for b in zip(self.breakpoints, self.breakpoint_enabled_states):
        b[0].enabled = b[1]
      if self.breakpoint_enabled_states[0] and self.log_publisher_state:
        print "Re-enabled ROS publisher."
    self.enabled_states_saved = False

  @staticmethod
  def convert_serialized_to_python(c_array, length):
    return ''.join([chr(c_array[char]) for char in range(length)])

  # build and publish map macro for cartographer, should be called
  # from cartographer_ros::Run stack frame
  def build_publish_map_cartographer(self):
    map_unique_ptr = gdb.parse_and_eval('node.map_builder_bridge_.BuildOccupancyGrid()')
    generated_map = map_unique_ptr['_M_t']['_M_head_impl'].dereference()

    self.handle_message(generated_map, None, '/map')
