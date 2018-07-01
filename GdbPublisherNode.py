import rospy
from rospy.msg import AnyMsg

class GdbPublisher(object):
  def __init__(self, topic, msgtype, queue_size = 1):
    self.topic = topic
    self.msgtype = msgtype
    # Construct publisher lazily, since we don't know whether to latch or not yet.
    self.publisher = None
    self.queue_size = queue_size

  def publish(self, serialized_message, latch):
    message = AnyMsg()
    message.deserialize(serialized_message)
    if self.publisher == None:
      self.publisher = rospy.Publisher(self.topic, self.msgtype, queue_size=self.queue_size, latch = latch)
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

  def publish(self, topic, serialized_message, latch):
    publisher = self.get(topic, None)

    if publisher != None:
      publisher.publish(serialized_message, latch)
      return True

    return False

class GdbMessageBreakpoint(gdb.Breakpoint):
  # the default context extractors
  default_serialized_message_extractor = lambda self: gdb.selected_frame().read_var('m')
  default_topic_extractor = lambda self: str(gdb.selected_frame().read_var('this').dereference()\
    ['name_']['_M_dataplus']['_M_p'].string())
  default_latched_extractor = lambda self: str(gdb.selected_frame().read_var('this').dereference()\
    ['latch_'])

  def __init__(self, location, context_extractor = lambda: {}):
    super(GdbMessageBreakpoint, self).__init__(location)
    self.context_extractor = context_extractor

  def stop(self):
    if(not hasattr(self, 'enclosing_node')):
      raise Exception('Missing node reference')

    context = self.context_extractor()
    if 'serialized_variable' not in context:
      context['serialized_variable'] = self.default_serialized_message_extractor()
    if 'topic' not in context:
      context['topic'] = self.default_topic_extractor()
    if 'latch' not in context:
      context['latch'] = self.default_latched_extractor()

    self.enclosing_node.handle_message(**context)
    return False

class GdbPublisherNode(object):
  def __init__ (self, publisher_dictionary, breakpoints, log_publisher = False, log_publisher_state = False):
    self.ros_handle = rospy.init_node('gdb_publisher')
    self.publisher_dictionary = publisher_dictionary
    self.breakpoints = []
    self.add_breakpoints(breakpoints)
    self.log_publisher = log_publisher
    self.log_publisher_state = log_publisher_state
    self.enabled_states_saved = False

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

  def handle_message(self, serialized_variable, topic, latch):
    if topic[0] == '/':
      topic = topic[1:]

    if topic not in self.publisher_dictionary:
      if self.log_publisher:
        print "didn't publish message in {}".format(topic)
      # don't bother with executing the rest since we are not publishing this message
      return

    message_start = serialized_variable['message_start']

    # in ros/serialization.h, serializeMessage(), num_bytes = len + 4
    serialized_length = serialized_variable['num_bytes'] - 4
    serialized_message = self.convert_serialized_to_python(message_start, serialized_length)

    self.publisher_dictionary.publish(topic, serialized_message, latch)
    if self.log_publisher:
      print 'published {}, {}'.format(topic, serialized_length)

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
    return
    if self.enabled_states_saved == True:
      raise Exception('This should not have happened, ROS publisher state has already been saved')
    self.breakpoint_enabled_states = [ b.enabled for b in self.breakpoints ]
    self.enabled_states_saved = True
    self.disable_breakpoints(False)
    if self.breakpoint_enabled_states[0] and self.log_publisher_state:
      print "Temporarily disabled ROS publisher."

  def restore_ros_publisher_state(self):
    return
    if self.enabled_states_saved == True:
      for b in zip(self.breakpoints, self.breakpoint_enabled_states):
        b[0].enabled = b[1]
      if self.breakpoint_enabled_states[0] and self.log_publisher_state:
        print "Re-enabled ROS publisher."
    self.enabled_states_saved = False

  @staticmethod
  def convert_serialized_to_python(c_array, length):
    return ''.join([chr(c_array[char]) for char in range(length)])
