#!/usr/bin/env python
import tf
from geometry_msgs.msg import PoseStamped

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import numpy as np
from sensor_msgs.msg import PointField
from nav_msgs.msg import Odometry

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.listener = tf.TransformListener()
        self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/limo/scan", LaserScan, self.laserCallback) #topic名称可能不同，请根据实际情况进行修改。可以在终端输入rostopic list查看

    # def laserCallback(self,data):
 
    #     cloud_out = self.laserProj.projectLaser(data)
    #     np_points = msg2np(cloud_out)
    #     odom_msg = rospy.wait_for_message("/odom_map", Odometry)
    #     robot_pose = np.array([odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z])

    #     # 复制十份并逐份增加0.5的z值
    #     np_points_copy = np.repeat(np_points, 10, axis=0)
    #     z_values = np.tile(np.arange(0, 10) * 0.2, len(np_points))  # 生成逐份增加的z值数组
    #     np_points_copy[:, 2] += z_values  # 将z值增加到复制的点云中

    #     cloud_out_modified = np2msg(np_points_copy)
    #     self.pcPub.publish(cloud_out_modified)
    #     # self.pcPub.publish(cloud_out)

    def laserCallback(self,data):
 
        cloud_out = self.laserProj.projectLaser(data)
        np_points = msg2np(cloud_out)
        odom_msg = rospy.wait_for_message("/odom_map", Odometry)
        robot_pose = np.array([odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z])

        distance = np.linalg.norm(np_points, axis=1)  # 计算每个点到原点的距离
        mask = distance >= 1  # 创建筛选条件
        np_points_filtered = np_points[mask]  # 筛选出距离大于等于1的点

        # 接下来进行复制操作
        np_points_copy = np.repeat(np_points_filtered, 10, axis=0)
        z_values = np.tile(np.arange(0, 10) * 0.2, len(np_points_filtered))
        np_points_copy[:, 2] += z_values

        cloud_out_modified = np2msg(np_points_copy)
        self.pcPub.publish(cloud_out_modified)


def np2msg(points):
    msg = PointCloud2()

    msg.header.stamp = rospy.Time().now()

    msg.header.frame_id = "laser_link"

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    # msg.is_dense = int(np.isfinite(points).all())
    msg.is_dense = False
    msg.data = np.asarray(points, np.float32).tostring()
    
    return msg
    
def msg2np(msg: PointCloud2, fileds=('x', 'y', 'z')):
    """
    激光雷达不同, msg 字节编码不同
    Args:
        msg:
        fileds_names:
    Returns: np.array, Nx3 或者 Nx4

    """

    def find_filed(filed):
        # 顺序查找
        for f in msg.fields:
            if f.name == filed:
                return f

    data_types_size = [None,
                    {'name': 'int8', 'size': 1},
                    {'name': 'uint8', 'size': 1},
                    {'name': 'int16', 'size': 2},
                    {'name': 'uint16', 'size': 2},
                    {'name': 'int32', 'size': 4},
                    {'name': 'uint32', 'size': 4},
                    {'name': 'float32', 'size': 4},
                    {'name': 'float64', 'size': 8}]

    dtypes_list = [None, np.int8, np.uint8, np.int16, np.uint16,
                np.int32, np.uint32, np.float32, np.float64]  # PointCloud2 中有说明

    np_list = []
    for filed in fileds:
        f = find_filed(filed)
        # print("f",f)
        dtype_size = data_types_size[f.datatype]['size']
        msg_total_type = msg.point_step

        item = np.frombuffer(msg.data, dtype=dtypes_list[f.datatype]).reshape(-1, int(
            msg_total_type / dtype_size))[:, int(f.offset / dtype_size)].astype(np.float32)
        np_list.append(item)

    points = np.array(np_list).T
    return points


if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()

    rate = rospy.Rate(0.2)  # 设置节点的执行频率为10Hz

    while not rospy.is_shutdown():
        # 在这里添加你的处理逻辑
        rate.sleep()

    rospy.spin()