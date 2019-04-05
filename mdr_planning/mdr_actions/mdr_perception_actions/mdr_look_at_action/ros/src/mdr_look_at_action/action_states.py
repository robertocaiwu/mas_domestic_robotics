#!/usr/bin/env python
import rospy
import numpy as np
import tf

from mdr_move_base_action.msg import MoveBaseGoal
from std_msgs.msg import String
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_find_object_action.msg import FindObjectGoal, FindObjectResult

class LookAtSM(ActionSMBase):
    def __init__(self,
                 object_position,
                 link_name,
                 number_of_retries=0,
                 timeout=120.,
                 max_recovery_attempts=1):
        super(FindObjectSM, self).__init__('FindObject', [], max_recovery_attempts)
        self._object_in_frustrum_pub = rospy.Publisher('/object_in_frustum',
                                                        String,
                                                        queue_size=1)
        self.object_position = object_position
        self.link_name = link_name
        self.tf_listener = tf.TransformListener()

        self.number_of_retries = number_of_retries
        self.timeout = timeout

        self.P = np.asarray([[538.12050153, 0., 320.19070834, 0.],
                             [0. ,538.81613509, 230.98657922, 0.],
                             [0. ,0. , 1., 0.]])


    def running(self):
        object_in_frustum = False
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_sensor_link = self.tf_listener.transformPose(self.link_name, pose)
        pose_sensor_link_vector = np.array([[pose_sensor_link.pose.orientation.x],
                                            [pose_sensor_link.pose.orientation.y],
                                            [pose_sensor_link.pose.orientation.z],
                                            [1]])

        image_projection.x,image_projection.y,_ = self.P.dot(pose_sensor_link_vector)

        if image_projection.x < 0:
            rospy.loginfo('The object is left')
        elif image_projection.x > 0 && image_projection.x < 640:
            rospy.loginfo('The object is in frustum in x')
        else:
            rospy.loginfo('The object is right')

        if image_projection.y < 0:
            rospy.loginfo('The object is on top')
        elif image_projection.y > 0 && image_projection.y < 480:
            rospy.loginfo('The object is in frustum in y')
        else:
            rospy.loginfo('The object is at the bottom')


        self.result = self.set_result(True, object_in_frustum)
        return FTSMTransitions.DONE

    def set_result(self, success, object_in_frustum):
        result = LookAtResult()
        result.success = success
        result.object_in_frustum = object_in_frustum
        return result
