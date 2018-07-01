from sensor_msgs import msg as sensor_msgs
from rosgraph_msgs import msg as graph_msgs
from nav_msgs import msg as nav_msgs
from tf2_msgs import msg as tf2_msgs
from genpy import rostime
from visualization_msgs import msg as visualization_msgs

# refer to README.md for more information

if 'gdb_publisher_node' not in locals():
  publisher_dictionary = GdbPublisherDictionary([
    GdbPublisher('clock', graph_msgs.Clock),
    GdbPublisher('scan_matched_points2', sensor_msgs.PointCloud2),
    GdbPublisher('tf', tf2_msgs.TFMessage),
    GdbPublisher('tf_static', tf2_msgs.TFMessage),
    GdbPublisher('constraint_list', visualization_msgs.MarkerArray)])

  breakpoints = [
  GdbMessageBreakpoint(location = 'gdb_ros_publisher_helper.cpp:Publication::publish')
 ]

  log_publishing = False
  log_publishing_state = True

  gdb_publisher_node = GdbPublisherNode(publisher_dictionary, breakpoints, log_publishing, log_publishing_state)
