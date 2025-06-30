#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.distance import cdist

class ClusterTracker:
    """聚类跟踪器，用于保持聚类的时间一致性"""
    def __init__(self, max_distance=1.0, min_lifetime=3, max_age=10):
        self.clusters = []  # 存储历史聚类信息
        self.max_distance = max_distance  # 聚类匹配的最大距离
        self.min_lifetime = min_lifetime  # 最小生存时间
        self.max_age = max_age  # 最大年龄
        self.next_id = 0
        
    def update(self, new_clusters):
        """更新聚类跟踪"""
        if not self.clusters:
            # 第一帧，直接初始化
            for cluster in new_clusters:
                self.clusters.append({
                    'id': self.next_id,
                    'center': cluster['center'],
                    'bbox': cluster['bbox'],
                    'size': cluster['size'],
                    'age': 0,
                    'matched_count': 1,
                    'history': [cluster]
                })
                self.next_id += 1
            return self.get_stable_clusters()
        
        # 计算距离矩阵进行匹配
        if new_clusters:
            old_centers = np.array([c['center'] for c in self.clusters])
            new_centers = np.array([c['center'] for c in new_clusters])
            distances = cdist(old_centers, new_centers)
            
            # 匈牙利匹配（简化版）
            matched_pairs = []
            for i in range(len(self.clusters)):
                if len(new_clusters) > 0:
                    min_dist_idx = np.argmin(distances[i])
                    if distances[i][min_dist_idx] < self.max_distance:
                        matched_pairs.append((i, min_dist_idx))
                        # 移除已匹配的新聚类
                        distances[:, min_dist_idx] = np.inf
            
            # 更新匹配的聚类
            matched_new_indices = set()
            for old_idx, new_idx in matched_pairs:
                matched_new_indices.add(new_idx)
                self.clusters[old_idx]['matched_count'] += 1
                self.clusters[old_idx]['age'] = 0
                self.clusters[old_idx]['history'].append(new_clusters[new_idx])
                
                # 保持历史记录长度
                if len(self.clusters[old_idx]['history']) > 5:
                    self.clusters[old_idx]['history'].pop(0)
                
                # 使用指数移动平均更新参数
                alpha = 0.3  # 平滑系数
                old_cluster = self.clusters[old_idx]
                new_cluster = new_clusters[new_idx]
                
                old_cluster['center'] = alpha * new_cluster['center'] + (1 - alpha) * old_cluster['center']
                old_cluster['size'] = int(alpha * new_cluster['size'] + (1 - alpha) * old_cluster['size'])
                
                # 边界框平滑
                old_bbox = old_cluster['bbox']
                new_bbox = new_cluster['bbox']
                old_bbox['min'] = alpha * new_bbox['min'] + (1 - alpha) * old_bbox['min']
                old_bbox['max'] = alpha * new_bbox['max'] + (1 - alpha) * old_bbox['max']
                old_bbox['size'] = old_bbox['max'] - old_bbox['min']
            
            # 添加新的未匹配聚类
            for i, cluster in enumerate(new_clusters):
                if i not in matched_new_indices:
                    self.clusters.append({
                        'id': self.next_id,
                        'center': cluster['center'],
                        'bbox': cluster['bbox'],
                        'size': cluster['size'],
                        'age': 0,
                        'matched_count': 1,
                        'history': [cluster]
                    })
                    self.next_id += 1
        
        # 增加未匹配聚类的年龄
        for cluster in self.clusters:
            if cluster['age'] < self.max_age:
                cluster['age'] += 1
        
        # 移除过老的聚类
        self.clusters = [c for c in self.clusters if c['age'] < self.max_age]
        
        return self.get_stable_clusters()
    
    def get_stable_clusters(self):
        """获取稳定的聚类"""
        stable_clusters = []
        for cluster in self.clusters:
            # 只返回存活时间足够长的聚类
            if cluster['matched_count'] >= self.min_lifetime:
                stable_clusters.append(cluster)
        return stable_clusters

class PointCloudClusteringNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pointcloud_clustering_node', anonymous=True)
        
        # 聚类参数
        self.eps = rospy.get_param('~eps', 0.4)  
        self.min_points = rospy.get_param('~min_points', 8)  
        self.max_points = rospy.get_param('~max_points', 1000)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 15)  # 
        
        # 点云预处理参数
        self.voxel_size = rospy.get_param('~voxel_size', 0.08)  # 增大体素大小
        self.z_min = rospy.get_param('~z_min', -0.8)
        self.z_max = rospy.get_param('~z_max', 1.5)
        self.radius_filter = rospy.get_param('~radius_filter', 5.0)
        
        # 聚类跟踪器
        self.tracker = ClusterTracker(
            max_distance=1.5,  # 匹配距离阈值
            min_lifetime=2,    # 最小生存帧数
            max_age=5         # 最大消失帧数
        )
        
        # 订阅者和发布者
        self.pc_subscriber = rospy.Subscriber('/livox/lidar', PointCloud2, self.pointcloud_callback, queue_size=1)
        self.cluster_publisher = rospy.Publisher('/pointcloud_clusters', MarkerArray, queue_size=1)
        self.filtered_pc_publisher = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=1)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("改进版点云聚类节点已启动")
        rospy.loginfo(f"聚类参数: eps={self.eps}, min_points={self.min_points}")
        rospy.loginfo(f"预处理参数: voxel_size={self.voxel_size}, z_range=[{self.z_min}, {self.z_max}]")

    def pointcloud_callback(self, msg):
        try:
            # 转换ROS PointCloud2到numpy数组
            points_list = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
            
            if len(points_list) == 0:
                rospy.logwarn("接收到空点云数据")
                return
            
            # 转换为numpy数组
            points = np.array(points_list, dtype=np.float32)
            
            # 使用Open3D进行点云处理和聚类
            raw_clusters = self.process_pointcloud(points)
            
            # 使用跟踪器更新聚类
            stable_clusters = self.tracker.update(raw_clusters)
            
            # 发布稳定的聚类结果
            if stable_clusters:
                self.publish_clusters(stable_clusters, msg.header)
                rospy.loginfo(f"检测到 {len(stable_clusters)} 个稳定聚类")
            
        except Exception as e:
            rospy.logerr(f"点云处理错误: {str(e)}")

    def process_pointcloud(self, points):
        """使用Open3D处理点云并进行聚类"""
        try:
            # 创建Open3D点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 1.体素下采样     
            if self.voxel_size > 0:
                pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

            # 2. 距离过滤（移除过远的点）
            center = np.array([0.0, 0.0, 0.0])
            distances = np.linalg.norm(np.asarray(pcd.points) - center, axis=1)
            pcd = pcd.select_by_index(np.where(distances < self.radius_filter)[0])
            
            # 3. Z轴范围过滤
            points_np = np.asarray(pcd.points)
            z_mask = (points_np[:, 2] >= self.z_min) & (points_np[:, 2] <= self.z_max)
            pcd = pcd.select_by_index(np.where(z_mask)[0])
            
            if len(pcd.points) < self.min_points:
                rospy.logwarn("过滤后点云过少，无法进行聚类")
                return []
            
            # 4. 移除离群点（使用更宽松的参数）
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=2.5)
            
            # 发布过滤后的点云用于可视化
            self.publish_filtered_pointcloud(pcd)
            
            # 5. DBSCAN聚类
            labels = np.array(pcd.cluster_dbscan(eps=self.eps, min_points=self.min_points))
            
            # 处理聚类结果
            clusters = []
            max_label = labels.max()
            
            for i in range(max_label + 1):
                cluster_indices = np.where(labels == i)[0]
                
                # 过滤小聚类
                if len(cluster_indices) < self.min_cluster_size:
                    continue
                    
                # 过滤大聚类（可能是噪声）
                if len(cluster_indices) > self.max_points:
                    continue
                
                cluster_points = np.asarray(pcd.points)[cluster_indices]
                
                # 计算更稳定的边界框
                bbox = self.compute_stable_bounding_box(cluster_points)
                
                clusters.append({
                    'points': cluster_points,
                    'center': np.mean(cluster_points, axis=0),
                    'size': len(cluster_indices),
                    'bbox': bbox
                })
            
            return clusters
            
        except Exception as e:
            rospy.logerr(f"Open3D处理错误: {str(e)}")
            return []

    def compute_stable_bounding_box(self, points):
        """计算更稳定的边界框"""
        # 使用百分位数而不是绝对最值，减少离群点影响
        percentile = 5  # 去除最外层5%的点
        
        min_bound = np.percentile(points, percentile, axis=0)
        max_bound = np.percentile(points, 100-percentile, axis=0)
        
        # 确保边界框有最小尺寸
        size = max_bound - min_bound
        min_size = 0.1
        for i in range(3):
            if size[i] < min_size:
                center = (min_bound[i] + max_bound[i]) / 2
                min_bound[i] = center - min_size / 2
                max_bound[i] = center + min_size / 2
        
        return {
            'min': min_bound,
            'max': max_bound,
            'size': max_bound - min_bound
        }

    def publish_filtered_pointcloud(self, pcd):
        """发布过滤后的点云"""
        try:
            points = np.asarray(pcd.points)
            if len(points) == 0:
                return
                
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "livox_frame"
            
            # 创建PointCloud2消息
            fields = [
                point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
                point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
                point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1),
            ]
            
            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.filtered_pc_publisher.publish(pc_msg)
            
        except Exception as e:
            rospy.logerr(f"发布过滤点云错误: {str(e)}")

    def publish_clusters(self, clusters, header):
        """发布聚类结果为MarkerArray"""
        try:
            marker_array = MarkerArray()
            
            for cluster in clusters:
                # 创建边界框标记
                marker = Marker()
                marker.header = header
                marker.header.stamp = rospy.Time.now()
                marker.ns = "clusters"
                marker.id = cluster['id']  # 使用稳定的ID
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # 设置边界框位置和大小
                center = cluster['center']
                size = cluster['bbox']['size']
                
                marker.pose.position.x = center[0]
                marker.pose.position.y = center[1]
                marker.pose.position.z = center[2]
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = max(size[0], 0.1)
                marker.scale.y = max(size[1], 0.1)
                marker.scale.z = max(size[2], 0.1)
                
                # 使用稳定的颜色（基于ID）
                colors = [
                    (1.0, 0.0, 0.0),  # 红
                    (0.0, 1.0, 0.0),  # 绿
                    (0.0, 0.0, 1.0),  # 蓝
                    (1.0, 1.0, 0.0),  # 黄
                    (1.0, 0.0, 1.0),  # 紫
                    (0.0, 1.0, 1.0),  # 青
                    (1.0, 0.5, 0.0),  # 橙
                    (0.5, 0.0, 1.0),  # 紫蓝
                ]
                color = colors[cluster['id'] % len(colors)]
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.6
                
                marker.lifetime = rospy.Duration(1.0)  # 增加生存时间
                marker_array.markers.append(marker)
                
                # 创建文本标记显示聚类信息
                text_marker = Marker()
                text_marker.header = header
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "cluster_text"
                text_marker.id = cluster['id'] + 10000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = center[0]
                text_marker.pose.position.y = center[1]
                text_marker.pose.position.z = center[2] + size[2]/2 + 0.5
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.4
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                text_marker.text = f"ID: {cluster['id']}\nPts: {cluster['size']}\nAge: {cluster.get('matched_count', 1)}"
                text_marker.lifetime = rospy.Duration(1.0)
                marker_array.markers.append(text_marker)
            
            self.cluster_publisher.publish(marker_array)
            
        except Exception as e:
            rospy.logerr(f"发布聚类标记错误: {str(e)}")

    def run(self):
        """运行节点"""
        rospy.loginfo("改进版点云聚类节点开始运行...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("节点关闭")

if __name__ == '__main__':
    try:
        node = PointCloudClusteringNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
