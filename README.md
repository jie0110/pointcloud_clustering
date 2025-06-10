# dbscan_clustering 点云聚类与跟踪节点

本项目基于 ROS、Open3D 和 DBSCAN 实现了激光雷达点云的高效聚类与跨帧目标跟踪，适用于机器人环境感知、动态目标检测等场景。

## 主要功能

- **点云预处理**：体素下采样、距离过滤、Z轴过滤、离群点剔除
- **DBSCAN聚类**：自适应密度的无监督聚类算法，自动识别点云中的目标
- **聚类稳定性跟踪**：跨帧目标ID一致性、属性平滑、鲁棒边界框
- **可视化**：发布聚类包围盒和文本信息，支持RViz显示
- **参数可调**：聚类与预处理参数均可通过ROS参数灵活配置

## 依赖

- ROS (建议Melodic/Noetic)
- Python 3
- open3d
- numpy
- scipy
- sensor_msgs, visualization_msgs, std_msgs
- tf2_ros

安装依赖示例：
```bash
pip install open3d numpy scipy
sudo apt install ros-<distro>-tf2-ros ros-<distro>-sensor-msgs ros-<distro>-visualization-msgs
```

## 启动方法

1. **拷贝脚本到ROS包下的scripts目录**  
2. **赋予可执行权限**  
   ```bash
   chmod +x dbscan_clustering.py
   ```
3. **启动节点**  
   ```bash
   rosrun <your_package_name> dbscan_clustering.py
   ```

## 主要参数

可通过ROS参数服务器设置，常用参数如下：

| 参数名             | 说明                 | 默认值   |
|--------------------|----------------------|----------|
| ~eps               | DBSCAN聚类半径       | 0.4      |
| ~min_points        | DBSCAN最小点数       | 8        |
| ~max_points        | 聚类最大点数         | 1000     |
| ~min_cluster_size  | 聚类最小点数         | 15       |
| ~voxel_size        | 体素下采样大小       | 0.08     |
| ~z_min             | Z轴最小值            | -0.8     |
| ~z_max             | Z轴最大值            | 1.5      |
| ~radius_filter     | 最大半径过滤         | 5.0      |

## 话题说明

- **订阅**  
  `/livox/lidar` (`sensor_msgs/PointCloud2`)  
  输入原始点云数据

- **发布**  
  `/filtered_pointcloud` (`sensor_msgs/PointCloud2`)  
  预处理后的点云（可用于RViz可视化）

  `/pointcloud_clusters` (`visualization_msgs/MarkerArray`)  
  聚类包围盒与文本信息（可用于RViz可视化）

## 稳定聚类跟踪原理

- 每一帧聚类后，使用中心点距离将新聚类与历史聚类进行匹配
- 匹配成功的聚类ID保持一致，并对中心、边界框等属性做指数移动平均平滑
- 新出现的聚类分配新ID，消失的聚类延迟若干帧后删除
- 只有连续多帧都被检测到的聚类才会被认为是“稳定聚类”并发布
- 边界框采用百分位数计算，减少离群点影响

## 可视化效果

- RViz中可看到每个聚类的包围盒（不同颜色）、ID、点数、存活帧数等信息
- 支持动态目标的稳定跟踪与显示

## 参考

- [Open3D官方文档](http://www.open3d.org/docs/release/)
- [DBSCAN算法介绍](https://scikit-learn.org/stable/modules/clustering.html#dbscan)
- ROS官方文档

---

如有问题欢迎提issue或交流！
