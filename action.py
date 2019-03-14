import rospy
from actionlib import simple_action_client
import move_base_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Search:

    def __init__(self):
        # publishers
        # this node will contain all of the subscribers and publishers
        # this will publish map navigation commands
        self.client = simple_action_client.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
        # this will publish twist commands to make the robot spin or stop
        self.twist_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_colour = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.colours_to_find = ['red', 'green', 'yellow', 'blue']



    def move_client(self, x,y):

        self.client.wait_for_server()

        self.goal = move_base_msgs.msg.MoveBaseGoal()

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1

        self.client.send_goal_and_wait(goal)


    def callback(self, data):
        cv2.namedWindow("Segmentation", 1)
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        
        if 'red' in self.colours_to_find:
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([5, 360, 360])
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            self.mask = red_mask

            # if it finds the colour, move towards it, stop within 1 meter and print found in the terminal
            # then remove this colour from the list
        
        if 'blue' in self.colours_to_find:
            lower_blue = np.array([100, 120, 100])
            upper_blue = np.array([120, 360, 360])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            if 'red' in self.colours_to_find:
                self.mask = self.mask + blue_mask
            else:
                self.mask = blue_mask
        
        if 'yellow' in self.colours_to_find:
            lower_yellow = np.array([20, 120, 100])
            upper_yellow = np.array([30, 360, 360])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            if 'red' in self.colours_to_find and 'blue' in self.colours_to_find:
                self.mask = self.mask + yellow_mask
            else:
                self.mask = yellow_mask
        
        if 'green' in self.colours_to_find:
            lower_green = np.array([50, 120, 100])
            upper_green = np.array([65, 360, 360])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            if 'red' in self.colours_to_find and 'blue' in self.colours_to_find and 'yellow' in self.colours_to_find:
                self.mask = self.mask + green_mask
            else:
                self.mask = green_mask


        # now apply a mask

        threshImg = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.mask)
        cv2.imshow("Segmentation",threshImg)
        cv2.waitKey(1)

         # Get the image shape, for calculating error
        h, w, d = self.cv_image.shape
        search_top = h / 4
        search_bot = 3*h/4 + 20
        self.mask[0:search_top, 0:w] = 0
        self.mask[search_bot:h, 0:w] = 0
        M = cv2.moments(self.mask)
        if M['m00'] > 0:  # If the area is greater than 0
            # find the x and y co-ordinates of the centroid of the region
            cx = int(M['m10']/M['m00']) 
            cy = int(M['m01']/M['m00'])

            # Calculate the error, or how much the robot needs to turn to get the object at the center of its vision
            error = cx - w/2
            # cancel the goal
            last_goal = self.goal
            self.client.cancel_goal()
            # Publish a twist command telling the robot to move to the pole
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = -float(error) / 100


            # now use the depth image to see how far the robot is from the object at the centroid calculated earlier
            # if the robot is under 1 meter away, stop
            cDepth = self.depth[cy, cx]
            if cDepth < 1.0:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                # get the colour of the object found
                print(threshImg[cy, cx][0:3])
                if np.all(threshImg[cy, cx] == [0, 102, 102]):
                    print("found yellow")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'yellow']
                    print(self.colours_to_find)
                if np.all(threshImg[cy, cx] == [0, 102, 0]):
                    print("found green")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'green']
                    print(self.colours_to_find)
                if np.all(threshImg[cy, cx] == [0, 0, 102]):
                    print("found red")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'red']
                    print(self.colours_to_find)

            self.twist_pub.publish(twist_msg)
            # republish the old goal
            self.client.send_goal(last_goal)
        


    def depth_image_callback(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def spin(self):
        rate = rospy.Rate(20)
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 4
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 4
        rospy.loginfo(twist_msg) # logs to terminal screen, but also to rosout and node log file
        self.twist_pub.publish(twist_msg)
        # rate.sleep()
        print("spinning")

rospy.init_node('move')
search = Search()

while search.colours_to_find:
    search.move_client(-4,5)
    search.move_client(-3,-3)
    search.move_client(-4,0)
    search.move_client(0,0)
    search.move_client(0,5)
    search.move_client(2,-5)
    search.move_client(0,0)