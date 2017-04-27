# unrealcv-ros

ROS bindings for UnrealCV (see [https://github.com/unrealcv/unrealcv](https://github.com/unrealcv/unrealcv])).
This enables the control of an UnrealEngine instance running UnrealCV using ROS.
Still a work in progress, these instructions will be updated as more is implemented.

## Usage

### Services

The package contains two different kinds of nodes, depending on how you want to use the simulator:
- `simple_node.py`: Defines a single service which takes a geometry_msgs/Pose input and returns a
sensor_msgs/Image output that is the scene at the given location 
- `passthrough_node.py`: A more complete node that has Services for the whole UnrealCV API
(see [http://unrealcv.org/reference/commands.html](http://unrealcv.org/reference/commands.html)) 

You can run either of these with `rosrun unrealcv_ros simple_node.py`

Only run one of these nodes at a time, they will compete for links to running UnrealCV instances.

### Example Clients

The package contains two example clients, one for each of the nodes described above.
They can be found in `example_simple_client.py` and `example_passthrough_client.py`.

## Dependencies
- ROS Kinetic: rospy
- UnrealCV: `pip install unrealcv`
- Numpy : `pip install numpy`

