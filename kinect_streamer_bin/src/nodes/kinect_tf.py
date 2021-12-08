import rospy
from fiducial_msgs.msg import *
import geometry_msgs
import tf
from tf.transformations import quaternion_matrix, quaternion_slerp
import numpy as np

weight = 0.95
class KinectTransform:
    def __init__(self):
        rospy.init_node("kinect_transform")
        self.transformations = [None] * 4
        self.prefixes = ["color_a", "color_b", "ir_a", "ir_b"]
        rospy.Timer(rospy.Duration(0.001), callback=self.callback_timer)
        for i in range(4):
            rospy.Subscriber("/" + self.prefixes[i] + "/fiducial_transforms", FiducialTransformArray, callback=self.callback, callback_args=i)

    def callback_timer(self, data):
        self.br = tf.TransformBroadcaster()
        for i in range(4):
            if self.transformations[i] is not None:
                self.br.sendTransform(self.transformations[i].translation, self.transformations[i].rotation, rospy.Time.now(), self.prefixes[i], "id1")


    def update(self, translation_meas, rotation_meas, i):
        if (self.transformations[i] is None):
            self.transformations[i] = geometry_msgs.msg.Transform()
            self.transformations[i].translation = translation_meas
            self.transformations[i].rotation = rotation_meas
            return
        self.transformations[i].translation = self.transformations[i].translation * weight + translation_meas * (1 - weight)
        self.transformations[i].rotation = quaternion_slerp(self.transformations[i].rotation, rotation_meas, 1.0 - weight)

    def callback(self, data, i):
        for fid_tf in data.transforms:
            id = fid_tf.fiducial_id
            if id != 1:
                continue
            transformations_meas = fid_tf.transform
            translation_meas = np.array([transformations_meas.translation.x, transformations_meas.translation.y, transformations_meas.translation.z])
            rotation_meas = np.array([transformations_meas.rotation.x, transformations_meas.rotation.y, transformations_meas.rotation.z, transformations_meas.rotation.w])
            R = quaternion_matrix(rotation_meas)[:3, :3]
            translation_meas = -R.T @ translation_meas
            rotation_meas[:3] *= -1
            self.update(translation_meas, rotation_meas, i)

if __name__ == "__main__":
    kinect_tf = KinectTransform()
    rospy.spin()