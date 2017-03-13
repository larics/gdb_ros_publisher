# gdb_ros_publisher
GDB support for republishing ROS messages in an rr replay

### Prerequisites
Be sure to have installed the newest build of rr and GDB compiled with Python 2 by following the instructions from [here](http://larics.fer.hr/farm/laricswiki/doku.php?id=software:debugging).


### Setting up
  1. Clone this repository into e.g. your home folder:
    
    ```bash
    git clone https://github.com/larics/gdb_ros_publisher
    ```
  2. Add the following to `.gdbinit` in your home directory. If you cloned the repository elsewhere, adjust the paths accordingly. 
    ```gdb
    define ros_publisher_setup
      source ~/gdb_ros_publisher/GdbPublisherNode.py
      source ~/gdb_ros_publisher/RunGdbPublisherNode.py
      source ~/gdb_ros_publisher/gdb_ros_publisher_setup
    end
    ```

  3. If necessary, customize the `RunGdbPublisherNode.py` file.
    
    1. Additional topics and message types can be set up by adding additional instances of `GdbPublisher` to the `GdbPublisherDictionary` instance. The class is constructed as follows:  

      ```python
      GdbPublisher(topic, msgtype, queue_size = 1)
      ```
    The arguments of the constructor are:
       * `topic`, string – topic name, without the leading slash
       * `msgtype` – a ROS message class type
       * `queue_size`, integer – ROS publisher queue size, optional (default: 1)
   
   2. The included breakpoints in `ros/publisher.h` should be enough to catch all published messages (except transforms). To add a new message publishing breakpoint, add a new `GdbMessageBreakpoint` instance to the `breakpoints` list. The constructor has the following prototype: 
   
      ```python
      GdbMessageBreakpoint(location, context_extractor = lambda: {})
      ```
    The arguments of the constructor are:
       * `location`, string – location in the source code from where the message will be extracted (it has to be completely initialized and ready for publishing at this point in the source code)
       * `context_extractor`, lambda function which returns a dictionary – this function is executed every time the breakpoint is reached, and computes a dictionary with extracted message variables. The following keys can be specified (all are optional, and default behavior is adapted for a breakpoint at the end of the function `template<M> void ros::Publisher::publish(const M &message)`):
         * `message_variable`, `gdb.Value` – contains the message. If unspecified, the default behavior is to look up a variable named `message`
         * `serialized_variable`, `gdb.Value`– the variable with the message already serialized. If there isn't one available, pass `None` in the dictionary (in that case, `GdbPublisherNode` will invoke ROS message serialization in order to transfer the message to Python). If unspecified, the default behavior is to look up a variable named `m`
         * `topic`, string – the topic on which the message is published; it has to match the topic in the appropriate `GdbPublisher`. The default behavior is to look up the topic name from the `this->impl_->topic_` string, where `this` is the `ros::Publisher` instance from the invocation of `ros::Publisher::publish()`

   3. To add a new transform publishing breakpoint, add a new `GdbTransformBreakpoint` instance to the `breakpoints` list. The constructor has the following prototype: 
      ```python
      GdbTransformBreakpoint(
        location, 
        transform_extractor = default_transform_extractor, 
        static_transform = False)
      ```
    The arguments of the constructor are:
       * `location`, string – location in the source code from where the stamped transform will be extracted (it has to be completely initialized and ready for publishing at this point in the source code)
       * `transform_extractor`, lambda function which returns a `gdb.Value` – this function is executed every time the breakpoint is reached, and computes the extracted stamped transform variable
         * if an extractor isn't passed, the default behavior is to look up a variable named `stamped_transform`
       * `static_transform`, boolean – whether to use a static transform broadcaster

#### Writing value extractors for variables

You can use the methods in the `gdb` module when writing variable extractors. Don't forget that they need to be inside a lambda function when instancing a publishing breakpoint, because they are executed every time the breakpoint is hit. Here are some examples:

  *  Get a variable named `message`:

      ```python
      gdb.selected_frame().read_var('message')
      ```

  *  Get a variable named `message`, which is a shared pointer in this particular case, and dereference it (in libstdc++, the pointer is stored in the `px` field of a shared pointer object; for a unique pointer, the pointer is located in `_M_t._M_head_impl`):

      ```python
      gdb.selected_frame().read_var('message')['px'].dereference()
      ```
  For accessing structure members, the `[]` Python operator is used on the `gdb.Value` object that represents a C++ structure.

  *  Get the string in `this->impl_->topic_`, where `impl_` is a shared pointer and `topic_` is a C++ string: 
    ```python
    str(gdb.selected_frame().read_var('this').dereference()\
        ['impl_']['px'].dereference()['topic_']['_M_dataplus']['_M_p'].string())
    ```
    The `string()` method of a `gdb.Value` (whose C++ type is `char *`) returns a Unicode string, so `str()` is finally used to convert it in a regular Python string.

  * You can also use `gdb.parse_and_eval('c++ expression')` to evaluate C++ expressions (such as function/method calls)

### Usage

  1. Run a `roscore` server and start a gdb debugging session.
  2. Call the `ros_publisher_setup` command in the gdb shell. 
  
  Afterwards, you can use the gdb commands `enable ros_publisher` and `disable ros_publisher` to enable/disable publishing (as with all gdb commands, you can use abbreviations, for example `ena ros` and `dis ros`).
