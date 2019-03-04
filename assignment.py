import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_colour = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        # self.image_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

    def callback(self, data):
        cv2.namedWindow("Segmentation", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
            numpy.array((0, 0, 50)),
            numpy.array((20, 20, 255)))

        threshImg = cv2.bitwise_and(cv_image, cv_image, mask=bgr_thresh)
        cv2.imshow("Segmentation", threshImg)
        cv2.waitKey(1)


class map_nav:
    def __init__(self):
        # publishers
        # this node will contain all of the subscribers and publishers
        # this will publish map navigation commands
        self.map_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # this will publish twist commands to make the robot spin or stop
        self.twist_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

    def move_and_spin(self):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 2.0
        goal.pose.position.y = 5.0
        goal.pose.position.z = 0.0


        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        rospy.sleep(1)

        self.map_pub.publish(goal)



rospy.init_node('navigation', anonymous=True)
map_nav1 = map_nav()
map_nav1.move_and_spin()
rospy.spin()
cv2.destroyAllWindows()

