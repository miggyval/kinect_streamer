import rospy
from fiducial_msgs.msg import *
import tf

def callback(data):
    for fid_tf in data.transforms:
        transform = fid_tf.transform
        id = fid_tf.fiducial_id
        br = tf.TransformBroadcaster()
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        br.sendTransform(translation, rotation, rospy.Time.now(), "id" + str(id), "world")

class KinectTransform:
    def __init__(self):
        rospy.init_node("kinect_transform")
        rospy.Subscriber("/color/fiducial_transforms", FiducialTransformArray, callback=callback)

if __name__ == "__main__":
    kinect_tf = KinectTransform()
    rospy.spin()