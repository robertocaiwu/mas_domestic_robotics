import rospy
import tf
from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import PointStamped
from mcr_perception_msgs.msg import PlaneList
from mdr_perception_libs import Constant, BoundingBox
from mdr_object_recognition.detection_service_proxy import DetectionServiceProxy


class ObjectDetector(object):
    def __init__(self, detection_service_proxy, event_out_topic):
        if not isinstance(detection_service_proxy, DetectionServiceProxy):
            raise ValueError('argument 1 is not a DetectionServiceProxy instance')

        self._detection_service_proxy = detection_service_proxy
        self._event_out_pub = rospy.Publisher(event_out_topic, StringMsg, queue_size=1)
        self._plane_list = None
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        pass

    def start_detect_objects(self, plane_frame_prefix, target_frame=None):
        self._plane_list = self._detection_service_proxy.get_objects_and_planes()
        if not isinstance(self._plane_list, PlaneList):
            raise ValueError('get_objects_and_planes() did not return a PlaneList instance')

        plane_index = 0
        for plane in self._plane_list.planes:
            # transform if target_frame is specified
            if target_frame is not None:
                # assuming only mcr_perception_msgs/Plane.pose is used
                plane.pose = self._transform_plane(plane.pose, target_frame)

            # publish plane frame
            plane_frame = '{0}_{1}'.format(plane_frame_prefix, plane_index)
            plane_index = plane_index + 1
            plane_pos = plane.pose.pose.position
            plane_quart = plane.pose.pose.orientation
            self._tf_broadcaster.sendTransform((plane_pos.x, plane_pos.y, plane_pos.z),
                                               (plane_quart.x, plane_quart.y, plane_quart.z, plane_quart.w),
                                               rospy.Time.now(), plane_frame, plane.pose.header.frame_id)

            # make bounding boxes
            normal = [plane_quart.x, plane_quart.y, plane_quart.z]
            for detected_obj in plane.object_list.objects:
                bounding_box = BoundingBox(detected_obj.pointcloud, normal)
                obj_pose = bounding_box.get_pose()
                bounding_box_msg = bounding_box.get_ros_message()
                if target_frame is not None:
                    bounding_box_msg, obj_pose = self._transform_object(bounding_box_msg, obj_pose, target_frame)
                    pass
                detected_obj.pose = obj_pose
                detected_obj.bounding_box = bounding_box_msg
                pass

        self._event_out_pub.publish(Constant.E_SUCCESS)
        return

    def _transform_plane(self, plane_pose, target_frame):
        try:
            common_time = self._tf_listener.getLatestCommonTime(target_frame, plane_pose.header.frame_id)
            plane_pose.header.stamp = common_time
            self._tf_listener.waitForTransform(target_frame, plane_pose.header.frame_id,
                                               plane_pose.header.stamp, rospy.Duration(0.1))

            plane_pose = self._tf_listener.transformPose(target_frame, plane_pose)
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to transform %s -> %s' % (plane_pose.header.frame_id, target_frame))

        return plane_pose

    def _transform_object(self, box_msg, obj_pose, target_frame):
        try:
            common_time = self._tf_listener.getLatestCommonTime(target_frame, obj_pose.header.frame_id)
            obj_pose.header.stamp = common_time
            self._tf_listener.waitForTransform(target_frame, obj_pose.header.frame_id,
                                               obj_pose.header.stamp, rospy.Duration(0.1))
            # transform object pose
            old_header = obj_pose.header
            obj_pose = self._tf_listener.transformPose(target_frame, obj_pose)
            # box center is the object position as defined in bounding_box_wrapper.cpp
            box_msg.center = obj_pose.pose.position

            # transform box vertices
            for vertex in box_msg.vertices:
                stamped = PointStamped()
                stamped.header = old_header
                stamped.point = vertex
                transformed = self._tf_listener.transformPoint(target_frame, stamped)
                vertex.x = transformed.point.x
                vertex.y = transformed.point.y
                vertex.z = transformed.point.z

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Unable to transform %s -> %s' % (obj_pose.header.frame_id, target_frame))

        return box_msg, obj_pose

    @property
    def plane_list(self):
        return self._plane_list
