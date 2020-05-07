import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes

def callback(img_sub, pc_sub):
  # The callback processing the pairs of numbers that arrived at approximately the same time
  print('callback')

def callback2(img_sub, pc_sub, bbox_sub):
    print('callback2.')

def debugCallback1(img_sub):
    print("debugCallback1")
    print(img_sub.header)

def debugCallback2(bbox_sub):
    print("debugCallback2")
    print(bbox_sub.header)

def main():
    rospy.init_node('SynchronizerNode_py_node', anonymous=False)
    print('SynchronizerNode_py_node start.')

    img_topic = "/darknet_ros/detection_image"
    img_bag_topic = "/camera/rgb/image_color"
    pc_topic = "/camera/republish/rgb/points"
    bbox_topic = "/darknet_ros/bounding_boxes"

    img_sub = message_filters.Subscriber(img_topic, Image)
    pc_sub = message_filters.Subscriber(pc_topic, PointCloud2)
    bbox_sub = message_filters.Subscriber(bbox_topic, BoundingBoxes)

    ts = message_filters.ApproximateTimeSynchronizer([img_sub, pc_sub], queue_size=10000, slop=100, allow_headerless=False)
    ts.registerCallback(callback)

    ts2 = message_filters.ApproximateTimeSynchronizer([img_sub, pc_sub, bbox_sub], queue_size=10000, slop=100, allow_headerless=True)
    ts2.registerCallback(callback2)

    #########

    # rospy.Subscriber(img_bag_topic, Image, debugCallback1)
    # rospy.Subscriber(bbox_topic, BoundingBoxes, debugCallback2)

    rospy.spin()

if __name__=='__main__':
    main()