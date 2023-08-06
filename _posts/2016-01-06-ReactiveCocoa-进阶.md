---
layout:     post
title:      Nav导航-代价地图
#subtitle:   
date:       2023-08-06
author:     BY
header-img: img/post-bg-ios9-web.jpg
catalog: true
tags:
    - ROS
    - Nav导航
    - 路径规划
    - 代价地图
---

## 前言
首先了解在导航堆中，move_base包与其它包（如amcl、map_server）的关系，如图所示
![nv](https://img-blog.csdnimg.cn/b7d2cb2863ed49db997ed0da4984a486.png)

发布导航命令

```cpp
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 60.0
    y: -6.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

导航过程中如果出现错误，终止导航命令

```cpp
rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
```

## 全局路径规划调试
可以使用动态调参工具（不会保存修改记录，每次修改自己作记录）[参考链接](https://blog.csdn.net/pricem/article/details/122891310?ops_request_misc=&request_id=&biz_id=102&utm_term=teb%E5%AF%BC%E8%88%AA%E4%B8%8D%E8%83%BD%E8%BF%87%E7%AA%84%E9%81%93&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-3-122891310.142%5Ev67%5Ewechat_v2,201%5Ev3%5Econtrol_2,213%5Ev2%5Et3_control1&spm=1018.2226.3001.4187)

```cpp
rosrun rqt_reconfigure rqt_reconfigure
```
为了更好调参，将cmd_vel话题重映射，即让小车不动，先调好全局路径，move_base.launch

```cpp
<remap from="/cmd_vel" to="/cmd_vel_1" />
```

## 什么是costmap？[参考链接](https://blog.csdn.net/shoufei403/article/details/104068107?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166986407416800180619183%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=166986407416800180619183&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-104068107-null-null.142%5Ev67%5Ewechat_v2,201%5Ev3%5Econtrol_2,213%5Ev2%5Et3_control1&utm_term=global%20costmap%20%E5%A2%99%E7%9A%84%E8%86%A8%E8%83%80%E8%B7%9D%E7%A6%BB&spm=1018.2226.3001.4187)
![costmap](https://img-blog.csdnimg.cn/5369c77a3b4c400a9e0012f70b9db1b5.png)
costmap翻译过来是代价地图的意思。由SLAM算法生成栅格地图。我们为栅格地图中的每一个栅格分配一个代价值，这样就形成了costmap。路径规划算法则可以在具有代价的栅格地图上生成路径。规划路径的生成则是强依赖于代价值。为了生成合适的路径，我们需要为每个栅格分配合适的代价值。最开始想到的是在单层的costmap中更新每个栅格的代价，然后直接给路径规划算法。但这样会引起诸多问题。比如因为所有的数据都在同一个costmap中更新，任何一个数据的变动都需要拿到之前其他的数据重新一起计算代价值。比如数据更新的地图范围也不好确定。比如当数据类型多了之后，数据整合的顺序不好控制。
后来想到将单层的costmap分成多层是个好办法。如上图所示，一层costmap只用同一种数据来更新。比如最底层的static map就是SLAM算法生成的静态地图。使用静态地图数据生成一层costmap。Obstacles 层则是由传感器数据更新的costmap层。甚至可以根据某些特殊目的自定义一个costmap层，使生成的路径规避某些区域。这在单层的costmap算法中是很难实现的。最后将所有的costmap层按特定的顺序组合起来形成了layered_costmap。可以看到这是一种更为灵活，扩展性也更强的方法。
*成本图参数的调整对于本地规划者的成功至关重要（不仅对于DWA）。在ROS中，代价地图由静态地图层、障碍地图层和膨胀层组成。静态地图层直接解释提供给导航堆栈的给定静态SLAM地图。障碍物地图层包括二维障碍和三维障碍（三维像素层）。膨胀层是一个障碍物膨胀的地方，用来计算每个二维成本图单元的成本。此外，还有一个global costmap和一个local costmap。global costmap是通过向导航点提供的地图上的障碍物膨胀而生成的。通过对机器人传感器实时检测到的障碍物进行膨胀，生成local costmap。*
*static map和Obstacles层都有一个costmap_，而layered_costmap类也维护了一个costmap_，并且这个costmap_最终组合了其他几个层的costmap_。而inflation层没有维护costmap_，它直接将cost值更新到了LayeredCostmap的costmap_里。*
*算法中仅使用footprint来计算inscribed_radius_（底盘内径）和circumscribed_radius_（底盘外径）。膨胀半径使用inflation_radius参数来设定*


## costmap_common_params.yaml [参考链接](https://blog.csdn.net/qq_42406643/article/details/118754093?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2~default~CTRLIST~Rate-1-118754093-blog-123262433.pc_relevant_3mothn_strategy_and_data_recovery&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2~default~CTRLIST~Rate-1-118754093-blog-123262433.pc_relevant_3mothn_strategy_and_data_recovery&utm_relevant_index=1)
该文件存储代价地图需要监听的传感器话题，代价地图公有参数等，全局代价地图和局部代价地图公有的参数放置进来，如此只需要载入这个配置文件就能完成共有参数的配置，避免了在全局代价地图和局部代价地图配置文件中重复配置一些相同参数的冗余。

**配置文件**

```cpp
# map_type: voxel/costmap
footprint: [[0.16, 0.12], [0.16,-0.12], [-0.16, -0.12], [-0.16, 0.12]]
obstacle_layer:
  enabled: true
  max_obstacle_height: 0.6
  min_obstacle_height: 0.0
  obstacle_range: 2.5
  raytrace_range: 3.0
  inflation_radius: 0.20
  combination_method: 1
  observation_sources: laser_scan_sensor 
  track_unknown_space: true
 
  origin_z: 0.0
  z_resolution: 0.1
  z_voxels: 10
  unknown_threshold: 15
  mark_threshold: 0
  publish_voxel_map: true
  footprint_clearing_enabled: true


  laser_scan_sensor:
    data_type: LaserScan
    topic: /scan
    marking: true
    clearing: true
    expected_update_rate: 0
    min_obstacle_height: 0.00
    max_obstacle_height: 0.30
inflation_layer:

  enabled: true

  cost_scaling_factor: 5.0

  inflation_radius: 0.2
static_layer:

  enabled: true
```
配置参数分为2类：机器人形状和代价地图各layer图层。代价地图各layer图层包括：静态层static_layer（由SLAM建立得到的地图提供数据）、障碍层obstacle_layer（由激光雷达等障碍扫描传感器提供实时数据）、膨胀层inflation_layer（在以上两层地图上进行膨胀（向外扩张），以避免机器人撞上障碍物）或者有voxel_layer体素层，Other Layers（你还可以通过插件的形式自己实现costmap，目前已有Social Costmap Layer、Range Sensor Layer等开源插件）。

 - 障碍物层obstacle_layer和体素层voxel_layer：这两层负责标注代价图上的障碍，他们可以被称为障碍层。障碍物层跟踪二维的，体素层跟踪三维的。障碍物是根据机器人传感器的数据进行检测或清除，其中需要订阅代价图的主题。在 ROS 执行中，体素层从障碍物层继承，并且都是通过使用激光雷达发布的 Point Cloud 或 PointCloud2类型的消息来获取障碍物信息。此外，体素层需要深度传感器，如 Microsoft Kinect 或华硕 Xtion 3D障碍物最终会被膨胀为二维代价图。
 - map_type: voxel 地图类型，这里为voxel(体素地图)。另一种地图类型为costmap(代价地图)。这两者之间的区别是前者是世界的3D表示，后者为世界的2D表示。
 - footprint：指机器人的轮廓，用于探知机器人是否能穿过某个障碍物（如门）。在ROS中，它由二维数组表示[x0,y0] ; [x1,y1] ; [x2,y2]（坐标的起点应该是机器人的中心，机器人的中心被认为是原点(0.0,0.0)，顺时针和逆时针都可以，不需要重复第一个坐标，单位是米）。该占位面积将用于计算内切圆和外接圆的半径，用于以适合此机器人的方式对障碍物进行膨胀。为了安全起见，通常将设置的稍大于机器人的实际轮廓。获取方式：（1）参考机器人的图纸。（2）自行绘制轮廓，然后选择一些顶点并使用标尺来确定它们的坐标。需要注意，x方向是机器人正直前进方向，给出footprint这几个点的时候要考虑到坐标系，要设置正确机器人的x轴。
![footprint](https://img-blog.csdnimg.cn/a531655d8a854147a11c9e9b542079c5.png)

 **obstacle_layer：配置障碍物图层**[参考链接](https://blog.csdn.net/luohuiwu/article/details/93653770?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166988101716782388068364%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166988101716782388068364&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-1-93653770-null-null.142%5Ev67%5Ewechat_v2,201%5Ev3%5Econtrol_2,213%5Ev2%5Et3_control1&utm_term=%E5%85%A8%E5%B1%80%E4%BB%A3%E4%BB%B7%E5%9C%B0%E5%9B%BE%E7%9A%84%E8%86%A8%E8%83%80%E5%8D%8A%E5%BE%84&spm=1018.2226.3001.4187)

                                                              
 - enabled：是否启用该层
 - combination_method：只能设置为0或1，用来更新地图上的代价值，一般设置为1
 - track_unknown_space:如果设置为false，那么地图上代价值就只分为致命碰撞和自由区域两种，如果设置为true，那么就分为致命碰撞，自由区域和未知区域三种
 - min/max_obstacle_height：插入代价图中的障碍物的最小/最大高度。该参数设置为稍高于机器人的高度。对于体素层，一般就是体素网格的高度
 - obstacle_range：设置机器人检测障碍物的最大范围。例如obstacle_range设为6.0，只有当机器人检测到一个距离小于6m的障碍物时，才会将这个障碍物引入到代价地图中。并可以在每个传感器的基础上进行覆盖
 - raytrace_range：用来设置机器人检测自由空间的最大范围。此参数用于机器人运动过程中，实时清除代价地图中的障碍物，例如设置为7.0，在机器人将根据传感器的信息，清除机器人前方7m远内障碍物信息，使得无障碍区域变为自由空间。以米为单位
 **这些参数仅用于体素层（VoxelCostMapPlugin）**
 - origin_z：地图的 Z 轴原点，以米为单位，仅对voxel地图
 - z_resolution：地图 Z 轴精度
 - z_voxels：每个垂直列中的体素数，网格的高度是 Z 轴分辨率*Z 轴体素数
 - unknown_threshold：当整列的voxel是“已知”(``known’’)的时候，含有的未知单元(“unknown”)的最大数量
 - mark_threshold：在被认为是“自由”的列中允许的标记单元的最大数量/整列voxel是“自由”(“free”)的时候，含有的已标记的cell(“marked”)的最大数目
 - publish_voxel_map: false #是否发布底层的体素栅格地图，其主要用于可视化
 **数据源**
 - **observation_sources:** scan bump # 观察源，我们这里是激光数据(scan)和凸点数据(bump)。观察源列表以空格分割表示，定义了下面参数中每一个 <source_name> 命名空间。**sensor_frame**: 参数应设置为传感器坐标帧的名称, **observation_persistence**: 设置传感器读数保存多长时间，单位为 seconds 。若设为0.0，则为保存最新读数信息。**expected_update_rate**: 读取传感器数据的频率，单位为 seconds 。若设为0.0，则为不断续地读取传感器数据。如果每0.05秒进行一次激光扫描，可以将此参数设置为0.1秒，以提供大量的缓冲区并占用一定的系统延迟, **max_obstacle_height**: 为传感器读数的最大有效高度，通常设置为略高于机器人的高度。将此参数设置为大于全局**max_obstacle_height**参数的值，则此参数无效；设置为小于全局**max_obstacle_height**的值将过滤掉高于该高度的该传感器的点, **min_obstacle_height**: 指传感器读数的最小有效高度，传感器读数的最小高度（以米为单位）被认为有效。这通常设置为地面高度，但可以根据传感器的噪声模型设置更高或更低, **data_type**: 参数应设置为LaserScan或PointCloud，这取决于主题使用的消息, **topic**: 设置为发布传感器数据的话题的名称，如/scan.**marking**: 是否将传感器数据用于向代价地图添加障碍物信息, **clearing**: 是否从代价地图清除障碍信息, **obstacle_range:**将障碍物插入代价地图的最大范围, **raytrace_range**: 从地图中扫描出障碍物的最大范围
***注意：***[参考链接](https://blog.csdn.net/u013468614/article/details/83386987?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166969211016800182137667%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=166969211016800182137667&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-83386987-null-null.142%5Ev67%5Ewechat,201%5Ev3%5Econtrol_2,213%5Ev2%5Et3_esquery_v1&utm_term=The%20origin%20for%20the%20sensor%20at%20%280.03,%200.03%29%20is%20out%20of%20map%20bounds.%20So,%20the%20costmap%20cannot%20raytrace%20for%20it.&spm=1018.2226.3001.4187)
   出现以下警告
   

```cpp
[ WARN] [1540470442.937371426, 3.000000000]: The origin for the sensor at (2.00, 1.98, 0.40) is out of map bounds. So, the costmap cannot raytrace for it.
[ WARN] [1540470443.935926596, 4.000000000]: The origin for the sensor at (2.00, 1.98, 0.40) is out of map bounds. So, the costmap cannot raytrace for it.
[ WARN] [1540470444.935980333, 5.000000000]: The origin for the sensor at (2.00, 1.98, 0.40) is out of map bounds. So, the costmap cannot raytrace for it.
[ WARN] [1540470445.936014597, 6.000000000]: The origin for the sensor at (2.00, 1.98, 0.40) is out of map bounds. So, the costmap cannot raytrace for it.

```
必须保证`z_voxels * z_resolution > max_obstacle_height`
**实验观察**
实验进一步阐明了体素层参数的影响。我们使用Asus Xtion Pro作为深度传感器。我们发现X离子的位置决定了“盲区”的范围，即深度传感器看不到任何东西的区域。

此外，仅当障碍物出现在局部代价地图更新范围内时，表示障碍物的体素才会更新（标记或清除）。否则，一些体素信息将保留，它们对Costmap膨胀的影响仍然存在。此外，Z分辨率控制体素在Z轴上的密度。如果它更高，体素层更密集。如果该值太低（例如0.01），所有体素将被组合在一起，因此您将无法获得有用Costmap信息。如果将Z分辨率设置为更高的值，您的目的应该是更好地获取障碍，因此需要增加Z体素参数，该参数控制多少个体素。在每个垂直列中。如果一列中的体素太多，但分辨率不够，这也是无用的，因为每个垂直列都有高度限制。

 **inflation_layer：配置膨胀层**
        *通过下图来认识下为何要设置膨胀层以及意义：*
        ![膨胀](https://img-blog.csdnimg.cn/a95b9c480dbc41ad84056ca9f4455f6a.png)


 - enabled：是否启用该层
 - cost_scaling_factor：膨胀过程中应用到代价值的比例因子，增大该比例因子会降低代价值
 - inflation_radius：膨胀半径，膨胀层会把障碍物代价膨胀直到该半径为止，一般将该值设置为机器人底盘的直径大小
 - inflation_radius 和 cost_scaling_factor 是决定膨胀的主要参数。inflation_radius 控制零成本点距离障碍物有多远，膨胀层会把障碍物的代价膨胀直到该半径为止。cost_scaling_factor 越大，膨胀点越宽，代价地图的边界越平滑，使得机器人越来越靠中间，远离墙边或障障碍物
**Pronobis博士建议**，最佳的Costmap衰减曲线是一条坡度相对较低的曲线，因此，最佳路径尽可能远离两侧的障碍物。其优点是机器人更喜欢在障碍物中间移动。如图8和图9所示，在相同的起点和目标下，当Costmap曲线陡峭时，机器人倾向于接近障碍物。在图14中，膨胀半径=0.55，成本比例系数=5.0；在图15中，膨胀半径=1.75，成本比例系数=2.58
![cost](https://img-blog.csdnimg.cn/2eca6d0635aa4bc9a961844788c97e5d.png)

![cost](https://img-blog.csdnimg.cn/bbedc9da8cf54e4cbd18719fc52e77ae.png)
在衰减曲线图的基础上，对这两个参数进行设置，使膨胀半径几乎覆盖了腐蚀体，且成本值衰减适中，这意味着降低了cost scaling factor的值。
## global_costmap_params.yaml

```cpp
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 1.0
  static_map: true
  
  rolling_window: false
  resolution: 0.05
  
  transform_tolerance: 1.0
  #inflation_radius: 0.1
  plugins:
     - {name: static_layer,            type: "costmap_2d::StaticLayer"}
     - {name: obstacle_layer,          type: "costmap_2d::ObstacleLayer"}
     - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}

```

 - global_frame：全局代价地图需要在哪个坐标系下运行
 - robot_base_frame：在全局代价地图中机器人本体的基坐标系。通过global_frame和robot_base_frame就可以计算两个坐标系之间的变换，得知机器人在全局坐标系global_frame中的坐标了
 - update_frequency：全局代价地图更新频率，一般全局代价地图更新频率设置的比较小，每个周期内，根据传感器信息mark/clear地图中的网格
 - publish_frequency：全局代价地图的发布频率，只用于Rviz可视化，这个参数没必要太大
 - static_map：配置是否使用map_server提供的地图来初始化，因为全局地图都是静态的，一般都设置为true
 - rolling_window：是否在机器人移动过程中需要滚动窗口，始终保持机器人在当前窗口中心位置，一般false
 - resolution：栅格地图的分辨率，该分辨率可以从加载的地图相对应的配置文件中获取到
 - inflation_radius：全局代价地图的膨胀半径
 - transform_tolerance：坐标系间的转换可以忍受的最大延时
 - plugins：在global_costmap中使用下面三个插件来融合三个不同图层，分别是static_layer、obstacle_layer和inflation_layer，合成一个master_layer来进行全局路径规划。**{name: static_layer, type: "costmap_2d::StaticLayer"}** 静态地图层
**{name: obstacle_layer, type: "costmap_2d::VoxelLayer"}** 障碍地图层
这里有个疑问：为什么obstacle_layer的类型不是普通的costmap_2d::ObstacleLayer？Turtlebot为什么将它设置为costmap_2d::VoxelLayer?其实在costmap_common_params.yaml也提到了，开启bump输入源是为了更好地可视化整个voxel层的情况，方便调试。所以这里将obstacle_layer的地图类型设置为costmap_2d::VoxelLayer。
**{name: inflation_layer, type: "costmap_2d::InflationLayer"}** 膨胀地图层，用于留出足够的安全距离


## local_costmap_params.yaml

```cpp
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 3.0
  publish_frequency: 3.0
  #static_map: false
  rolling_window: true
  width: 2.5
  height: 2.5
  resolution: 0.05
  plugins: 
  - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"} 
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

```
 

 - global_frame：在局部代价地图中的全局坐标系，一般需要设置为odom_frame
 - robot_base_frame：机器人本体的基坐标系
 - update_frequency：局部代价地图的更新频率，每个周期内，根据传感器信息mark/clear地图中的网格
 - publish_frequency：局部代价地图的发布频率，只用于Rviz可视化，这个参数没必要太大
 - static_map：局部代价地图一般不设置为静态地图，因为需要检测是否在机器人附近有新增的动态障碍物
 - rolling_window：使用滚动窗口，始终保持机器人在当前局部地图的中心位置
 - width：代价地图（滚动窗口）的宽度，单位米
 - resolution：代价地图（滚动窗口）的分辨率，注意，这里的分辨率可以不同于所建地图的分辨率，但一般情况下该参数的值与所建的地图一致
 - transform_tolerance：局部代价地图中的坐标系之间转换的最大可忍受延时
 - plugins：在局部代价地图中，不需要静态地图层，因为我们使用滚动窗口来不断的扫描障碍物，所以就需要融合两层地图（inflation_layer和obstacle_layer）即可，融合后的地图用于进行局部路径规划
**costmap resolution**
      本地代价地图和全局代价地图可以分别设置此参数。它们影响计算负载和路径规划。当低分辨率（>=0.05）时，在狭窄的通道中，障碍区域可能会重叠，因此local planner无法找到通过的路径。
      对于全局代价地图分辨率，它足以保持与提供给导航堆栈的地图的分辨率相同。如果你有足够的计算能力，你应该看看激光扫描仪的分辨率，因为当使用gmapping创建地图，如果激光扫描仪的分辨率低于您所需的地图分辨率，将会有很多小的“未知点”，因为激光扫描仪无法覆盖该区域，如图16所示。
      ![cost](https://img-blog.csdnimg.cn/b1887634d7b349a3b189d1a59e6365a9.png)
例如，Hokuyo URG-04LX-UG01激光扫描仪的公制分辨率为0.01mm。因此，扫描分辨率<=0.01的地图需要机器人旋转几次以清除未知的点。我们发现0.02是一个足够的解决方案。
      

      
   

 

