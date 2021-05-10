# Path planner in occupancy grid

## Goal

The goal of the process "path planner in occupancy grid" is to generate an obstacle-free path using a 2D representation of the environment map as an occupancy grid.

## Services

-   **generate_path** ([aerostack_msgs/GeneratePath](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/9a0258c56c201ebb8df010d2bbafce4871d2dedc/srv/GeneratePath.srv))  
    Generate a new path to reach a spatial point given as input.

## Subscribed topics

-   **tf** ([tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html))  
    The current pose of the drone.

-   **map** ([nav_msgs/GetMap](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html))   
    Environment map represented as occupancy grid.

-   **scan** ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))   
    Laser scan from lidar sensors.

    
## Published topics


-   **path_with_id** ([aerostack_msgs/PathWithID](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/9a0258c56c201ebb8df010d2bbafce4871d2dedc/msg/PathWithID.msg))
    Generated path with a numerical id.

![path_planner_in_occupancy_grid.png](https://bitbucket.org/repo/rokr9B/images/2237164571-path_planner_in_occupancy_grid.png)

## General approach

This process uses the planner "move_base", provided as a general ROS package. To install "move_base" from ROS write the following command (in this example we are using the ROS distribution ros-melodic, but you can use another distribution instead):

    $ sudo apt-get install ros-melodic-move-base*

The component "move_base" is a ROS node that uses as input the desired point to reach in the following topic:

-   **move_base_simple/goal** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  

The node "move_base" uses a plugin called "navfn" as a global planner (a grid-based global planner with a navigation function). For that, the parameter "base_global_planner" of "move_base" has the value "navfn/NavfnROS" (default value). This plugin publishes a message every time the planner computes a new path, in the following topic:

-   **move_base/NavfnROS/plan** ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))  

The node "move_base" also uses a plugin for the local planner. This plugin runs inside the "move_base" process. In this version, we have selected the default local planner, although its output is not used.

## Algorithm


The process path_planner_in_occupancy_grid receives as input target goals. This process uses a queue of goals and dispatches them one by one to the "move_base" planner. 

The use of this queue is done for better organization and to use timeout. 

The timeout is necessary because "move_base" does not work properly (e.g., when the computation power is low, sometimes "move_base" fails to generate a path). When the timeout is reached, it is assumed that "move_base" has failed, and the goal is re-sent. After doing this, the timeout is reset. 

The timeout is implemented with a thread and mutex is used to manage concurrency.

## References

ROS package "move_base": http://wiki.ros.org/move_base

ROS package "navfn": http://wiki.ros.org/navfn

---
# Contributors

**Code maintainer:** Raúl Cruz

**Authors:** Guillermo Echegoyen, Raúl Cruz
