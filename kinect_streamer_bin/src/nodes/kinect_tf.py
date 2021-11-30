import rospy
from fiducial_msgs.msg import *
import tf

def callback_a(data):
    for fid_tf in data.transforms:
        transform = fid_tf.transform
        id = fid_tf.fiducial_id
        br = tf.TransformBroadcaster()
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        br.sendTransform(translation, rotation, rospy.Time.now(), "world", "id" + str(id) + "a")

def callback_b(data):
    for fid_tf in data.transforms:
        transform = fid_tf.transform
        id = fid_tf.fiducial_id
        br = tf.TransformBroadcaster()
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        br.sendTransform(translation, rotation, rospy.Time.now(), "world", "id" + str(id) + "b")


class KinectTransform:
    def __init__(self):
        rospy.init_node("kinect_transform")
        rospy.Subscriber("/color_a/fiducial_transforms", FiducialTransformArray, callback=callback_a)
        rospy.Subscriber("/color_b/fiducial_transforms", FiducialTransformArray, callback=callback_b)

if __name__ == "__main__":
    kinect_tf = KinectTransform()
    rospy.spin()