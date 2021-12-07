import rospy
from fiducial_msgs.msg import *
import geometry_msgs
import tf
from tf.transformations import quaternion_matrix, quaternion_slerp
import numpy as np

weight = 0.9
class KinectTransform:
    def __init__(self):
        rospy.init_node("kinect_transform")
        self.transformation_a = None
        self.transformation_b = None
        rospy.Timer(rospy.Duration(0.001), callback=self.callback_timer)
        rospy.Subscriber("/color_a/fiducial_transforms", FiducialTransformArray, callback=self.callback_a)
        rospy.Subscriber("/color_b/fiducial_transforms", FiducialTransformArray, callback=self.callback_b)

    def callback_timer(self, data):
        br = tf.TransformBroadcaster()
        if self.transformation_a is not None:
            br.sendTransform(self.transformation_a.translation, self.transformation_a.rotation, rospy.Time.now(), "color_a", "id1")
        if self.transformation_b is not None:
            br.sendTransform(self.transformation_b.translation, self.transformation_b.rotation, rospy.Time.now(), "color_b", "id1")


    
    def update_a(self, translation_meas_a, rotation_meas_a):
        if (self.transformation_a is None):
            self.transformation_a = geometry_msgs.msg.Transform()
            self.transformation_a.translation = translation_meas_a
            self.transformation_a.rotation = rotation_meas_a
            return
        if np.linalg.norm(self.transformation_a.translation - translation_meas_a) < 1.0:
            self.transformation_a.translation = self.transformation_a.translation * weight + translation_meas_a * (1 - weight)
            self.transformation_a.rotation = quaternion_slerp(self.transformation_a.rotation, rotation_meas_a, weight)

    def update_b(self, translation_meas_b, rotation_meas_b):
        if (self.transformation_b is None):
            self.transformation_b = geometry_msgs.msg.Transform()
            self.transformation_b.translation = translation_meas_b
            self.transformation_b.rotation = rotation_meas_b
            return
        if np.linalg.norm(self.transformation_b.translation - translation_meas_b) < 1.0:
            self.transformation_b.translation = self.transformation_b.translation * weight + translation_meas_b * (1 - weight)
            self.transformation_b.rotation = quaternion_slerp(self.transformation_b.rotation, rotation_meas_b, weight)
        

    def callback_a(self, data):
        self.update_a(np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
        pass
        for fid_tf in data.transforms:
            id = fid_tf.fiducial_id
            if id != 1:
                continue
            transform_meas_a = fid_tf.transform
            translation_meas_a = np.array([transform_meas_a.translation.x, transform_meas_a.translation.y, transform_meas_a.translation.z])
            rotation_meas_a = np.array([transform_meas_a.rotation.x, transform_meas_a.rotation.y, transform_meas_a.rotation.z, transform_meas_a.rotation.w])
            R = quaternion_matrix(rotation_meas_a)[:3, :3]
            translation_meas_a = -R.T @ translation_meas_a
            rotation_meas_a[:3] *= -1
            self.update_a(translation_meas_a, rotation_meas_a)


    def callback_b(self, data):
        self.update_b(np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
        pass
        for fid_tf in data.transforms:
            id = fid_tf.fiducial_id
            if id != 1:
                continue
            transform_meas_b = fid_tf.transform
            translation_meas_b = np.array([transform_meas_b.translation.x, transform_meas_b.translation.y, transform_meas_b.translation.z])
            rotation_meas_b = np.array([transform_meas_b.rotation.x, transform_meas_b.rotation.y, transform_meas_b.rotation.z, transform_meas_b.rotation.w])

            R = quaternion_matrix(rotation_meas_b)[:3, :3]
            translation_meas_b = -R.T @ translation_meas_b
            rotation_meas_b[:3] *= -1
            self.update_b(translation_meas_b, rotation_meas_b)

if __name__ == "__main__":
    kinect_tf = KinectTransform()
    rospy.spin()