import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




class map_nav:
    def __init__(self):
        # publishers
        # this node will contain all of the subscribers and publishers
        # this will publish map navigation commands
        self.map_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # this will publish twist commands to make the robot spin or stop
        self.twist_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_colour = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.colours_to_find = ['red', 'green', 'yellow', 'blue']

    def move_and_spin(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0


        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        rospy.sleep(1)

        self.map_pub.publish(goal)


    def callback(self, data):
        cv2.namedWindow("Segmentation", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # bgr_thresh = cv2.inRange(cv_image,
            # np.array((0, 0, 50)),
            # np.array((20, 20, 255)))
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        if 'red' in self.colours_to_find:
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([5, 360, 360])
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            mask = red_mask

            # if it finds the colour, move towards it, stop within 1 meter and print found in the terminal
            # then remove this colour from the list
        
        if 'blue' in self.colours_to_find:
            lower_blue = np.array([100, 120, 100])
            upper_blue = np.array([120, 360, 360])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            mask = mask + blue_mask
        
        if 'yellow' in self.colours_to_find:
            lower_yellow = np.array([20, 120, 100])
            upper_yellow = np.array([30, 360, 360])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask = mask + yellow_mask
        
        if 'green' in self.colours_to_find:
            lower_green = np.array([50, 120, 100])
            upper_green = np.array([65, 360, 360])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            mask = mask + green_mask


        # now apply a mask

        threshImg = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        cv2.imshow("Segmentation", threshImg)
        cv2.waitKey(1)

    def spin(self):
        rate = rospy.Rate(50)
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 4
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 4
        rospy.loginfo(twist_msg) # logs to terminal screen, but also to rosout and node log file
        self.twist_pub.publish(twist_msg)
        rate.sleep()

    def depth_image_callback(self, data):
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data, "32FC1")






rospy.init_node('navigation', anonymous=True)
map_nav1 = map_nav()
map_nav1.move_and_spin(2.0, -5.0)
map_nav1.spin()
rospy.sleep(30)
map_nav1.move_and_spin(-4.0, 0.0)
map_nav1.spin()
rospy.sleep(30)
map_nav1.move_and_spin(0.0, 0.0)
map_nav1.spin()
rospy.spin()
cv2.destroyAllWindows()
