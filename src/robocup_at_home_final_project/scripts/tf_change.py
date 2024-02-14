#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

def transform_point(listener, camera_point):
    try:
        # 等待直到变换可用
        listener.waitForTransform("base_footprint", camera_point.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
        
        # 执行变换
        transformed_point = listener.transformPoint("base_footprint", camera_point)
        return transformed_point
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF Transform Error: %s" % e)
        return None

if __name__ == '__main__':
    rospy.init_node('camera_to_base_transformer')

    # 创建 tf 监听器
    listener = tf.TransformListener()

    # 创建一个 PointStamped 消息，代表要转换的点
    camera_point = geometry_msgs.msg.PointStamped()
    camera_point.header.frame_id = "xtion_rgb_optical_frame"
    # camera_point.header.stamp = rospy.Time.now()
    camera_point.point.x = -0.093
    camera_point.point.y = -0.004
    camera_point.point.z = 0.858

    rospy.sleep(2.0)  # 等待 tf 监听器初始化

    # 转换点并打印结果
    transformed_point = transform_point(listener, camera_point)
    if transformed_point is not None:
        rospy.loginfo("Transformed Point: x=%f, y=%f, z=%f" % (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))
