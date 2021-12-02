# Navigation Modules
We leverage ROS as the platform to communicate massages among navigation modules.
These modules should be executed on AGV.
## Module
* ydlidar
    Activate lidar and publish sensor data to ROS
* dashgo_driver
    Control the base of AGV
* dashgo_drscription
    Publish the AGV status
* map_server
    Visualize the map and sensor information
* multiGoalListen.py
    Listen goal and transport the data to ROS platform
* amcl
    Robot localization.
    It subscribes the topic of lidar, and decides where the robot is now based on monte carlo approach (KLD-sampling).
    [ROS Doc](http://wiki.ros.org/amcl)
    [KLD-Sampling](https://proceedings.neurips.cc/paper/2001/file/c5b2cebf15b205503560c4e8e6d1ea78-Paper.pdf)
* teb_move_base
    Path planning
    It reads map information and localization (robot position). In order to make a efficient and precise path, it will create a global costmap and local costmap based on the map provided and make global and local planning.
    ![](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf.png)
    [ROD Doc](http://wiki.ros.org/move_base)