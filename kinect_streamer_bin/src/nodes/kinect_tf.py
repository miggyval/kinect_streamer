import rospy
from fiducial_msgs.msg import *
import tf

def callback_a(data):
    for fid_tf in data.transforms:
        transform = fid_tf.transform
        id = fid_tf.fiducial_id
        if id not in [1, 2]:
            continue
        br = tf.TransformBroadcaster()
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        parent = "world"
        child = "id" + str(id) + "a"
        br.sendTransform(translation, rotation, rospy.Time.now(), parent, child)

def callback_b(data):
    for fid_tf in data.transforms:
        transform = fid_tf.transform
        id = fid_tf.fiducial_id
        if id not in [1, 2]:
            continue
        br = tf.TransformBroadcaster()
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        parent = "world"
        child = "id" + str(id) + "b"
        br.sendTransform(translation, rotation, rospy.Time.now(), parent, child)


class KinectTransform:
    def __init__(self):
        rospy.init_node("kinect_transform")
        rospy.Subscriber("/color_a/fiducial_transforms", FiducialTransformArray, callback=callback_a)
        rospy.Subscriber("/color_b/fiducial_transforms", FiducialTransformArray, callback=callback_b)

if __name__ == "__main__":
    kinect_tf = KinectTransform()
    rospy.spin()