from sensor_msgs import msg as sensor_msgs
from rosgraph_msgs import msg as graph_msgs
from nav_msgs import msg as nav_msgs
from tf2_msgs import msg as tf2_msgs
from genpy import rostime


if 'gdb_publisher_node' not in locals():
  publisher_dictionary = GdbPublisherDictionary([
    GdbPublisher('clock', graph_msgs.Clock),
    GdbPublisher('bag_scan', sensor_msgs.LaserScan),
    GdbPublisher('bag_odom', nav_msgs.Odometry),
    GdbPublisher('scan_matched_points2', sensor_msgs.PointCloud2),
    GdbPublisher('map', nav_msgs.OccupancyGrid),
    GdbPublisher('tf', tf2_msgs.TFMessage)])

  breakpoints = [
  GdbMessageBreakpoint(
    location = 'ros/publisher.h:89',
    context_extractor = 
      lambda: {'message_variable': gdb.selected_frame().read_var('message')['px'].dereference()} ),
  GdbMessageBreakpoint(
    location = 'ros/publisher.h:119'),
  GdbTransformBreakpoint(
    location = 'node.cc:161') ]

  log_publishing = True
  log_publishing_state = True

  gdb_publisher_node = GdbPublisherNode(publisher_dictionary, breakpoints, log_publishing, log_publishing_state)
