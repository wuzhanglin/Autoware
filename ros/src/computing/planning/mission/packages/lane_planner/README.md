# lane_planner

## Package

This package has following nodes.
- lane_navi
- lane_rule
- lane_select
- lane_stop

## Nodes

### lane_select

1. 概要
    1. 接受多个路径
    1. 对所有的路径，从当前位置得出最近傍点
    1. 最近在最靠近傍点的车道上运行的路径和设定
    1. 检测出当前路径的左右路径
    1. 将当前路径的最近傍点所具有的车道变更标志保持为该路径的车道变更标志
    1. 寻找最近右转或左转的标志的点，将该点与进行车道改变的路径的目标点进行艾尔米特插值后的路径，将该路径与车道变更的目标点以后的路径结合后的路径定义为用于车道变更的路径。
    1. 在不进行车道变更的情况下，使当前路径、其最近邻、车道变更标志分别publish。
    1. 在进行车道变更的情况下，publish分别进行车道变更用的路径、其最近傍点、车道变更标志。
    
1. 注意
    - 为了进行车道变更，需要读取多个ver3格式的路径文件。(参照waypoint_maker包装)

1. Subscribed Topics

    - traffic_waypoints_array (waypoint_follower/LaneArray)
    - current_pose (geometry_msgs/PoseStamped)
    - current_velocity (geometry_msgs/TwistStamped)
    - state (std_msgs/String)
    - config/lane_select (runtime_manager/ConfigLaneSelect)
    
1. Published Topics

    - base_waypoints (waypoint_follower/lane)
    - closest_waypoint (std_msgs/Int32)
    - change_flag (std_msgs/Int32)
    - lane_select_marker (visualization_msgs/MarkerArray) : for debug
    
1. Parameter from Runtime Manager

    - Distance threshold to neighbor lanes<br>
    表示检测当前路径周围的有效车道时的阈值。在比该阈值远的距离上的车道不被识别为车道。
    
    - Lane Change Interval After Lane Merge
    表示在进行了车道变更之后跑几米之后还能够进行车道变更的值。
    
    - Lane Change Target Ratio
    将待改变车道的车道上的目标点定义为与速度(m/s)成比例的距离时使用的值。
    目标点探索的起点是在计划变更车道的车道上的点，具有右转或左转的车道变更标志的点的最近傍点。
    
    - Lane Change Target Minimum
    表示到进行车道变更的预定的车道上的目标点为止的最低距离。
    目标点探索的起点是在计划变更车道的车道上的点，具有右转或左转的车道变更标志的点的最近傍点。
    
    - Vector Length of Hermite Curve
    表示用环形曲线补充时的矢量的大小。
