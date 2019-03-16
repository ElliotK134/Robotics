import rospy
from actionlib import simple_action_client
import move_base_msgs.msg
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import time
from nav_msgs.msg import OccupancyGrid


class Search:

    def __init__(self):
        # publishers
        # this node will contain all of the subscribers and publishers
        # this will publish map navigation commands
        self.client = simple_action_client.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
        # this will publish twist commands to make the robot spin or stop
        self.twist_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=40)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_colour = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.colours_to_find = ['red', 'green', 'yellow', 'blue']
        self.goal = move_base_msgs.msg.MoveBaseGoal()
        self.odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.orientation = 0
        self.mapsub = rospy.Subscriber('map', OccupancyGrid, self.map_cb)
        self.map = []
        self.x = 0
        self.y = 0

    def move_client(self, x, y):
        self.x = x
        self.y = y

        self.client.wait_for_server()

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1

        print("goal published")
        self.client.send_goal(self.goal, done_cb=self.done_cb)
        self.client.wait_for_result()
        print("goal completed")

    def done_cb(self, data, result):
        print("data:", data)
        if data == 4:
            # if the goal fails, use the self.x and y to get the last goal
            # then move it slightly and re send
            newx = self.x + 0.5
            newy = self.y + 0.5
            # check that the new co ordinates are not out of bounds
            if self.x > 4:
                self.x = 4
            if self.y > 5:
                self.y = 5
            if self.x < -4:
                self.x = -4
            if self.y < -5:
                self.y = -5
            # re send the goal
            self.move_client(newx, newy)
            # spin to look for pole
            print("sent new goal to", newx, newy)
            self.spin()
            print("exiting done_cb")

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
            if 'red' in self.colours_to_find or 'blue' in self.colours_to_find:
                self.mask = self.mask + yellow_mask
            else:
                self.mask = yellow_mask

        if 'green' in self.colours_to_find:
            lower_green = np.array([50, 120, 100])
            upper_green = np.array([65, 360, 360])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            if 'red' in self.colours_to_find or 'blue' in self.colours_to_find or 'yellow' in self.colours_to_find:
                self.mask = self.mask + green_mask
            else:
                self.mask = green_mask

        # now apply a mask

        self.threshImg = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.mask)
        cv2.imshow("Segmentation", self.threshImg)
        cv2.waitKey(1)

        h, w, d = self.cv_image.shape
        search_top = h / 4
        search_bot = 3 * h / 4 + 20
        self.mask[0:search_top, 0:w] = 0
        self.mask[search_bot:h, 0:w] = 0
        M = cv2.moments(self.mask)
        if M['m00'] > 0:  # If the area is greater than 0
            # find the x and y co-ordinates of the centroid of the region
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # now use the depth image to see how far the robot is from the object at the centroid calculated earlier
            # if the robot is under 1 meter away, stop
            depth = self.depth[cy, cx]
            if depth < 1.0:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                # get the colour of the object found
                print(self.threshImg[cy, cx][0:3])
                if np.all(self.threshImg[cy, cx] == [0, 102, 102]):
                    print("found yellow")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'yellow']
                    print(self.colours_to_find)
                if np.all(self.threshImg[cy, cx] == [0, 102, 0]):
                    print("found green")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'green']
                    print(self.colours_to_find)
                if np.all(self.threshImg[cy, cx] == [0, 0, 102]):
                    print("found red")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'red']
                    print(self.colours_to_find)
                if np.all(self.threshImg[cy, cx] == [102, 0, 0]):
                    print("found blue")
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'blue']
                    print(self.colours_to_find)
                self.twist_pub.publish(twist_msg)
                rospy.sleep(2)

    def depth_image_callback(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def spin_and_search(self):
        # publish an empty goal so nothing happens with action client in the mean time
        # self.client.send_goal_and_wait(move_base_msgs.msg.MoveBaseGoal())
        self.client.cancel_all_goals()
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 1
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1
        t = int(time.time())
        print("spinning")
        while int(time.time()) < t + 10:
            h, w, d = self.cv_image.shape
            search_top = h / 4
            search_bot = 3 * h / 4 + 20
            self.mask[0:search_top, 0:w] = 0
            self.mask[search_bot:h, 0:w] = 0
            M = cv2.moments(self.mask)
            if M['m00'] > 5000:  # If the area is greater than 0
                twist_msg.angular.z = 0
                self.twist_pub.publish(twist_msg)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # try to send a move command to the pole
                # first find the depth
                depth = self.depth[cy, cx]
                if not math.isnan(depth):
                    self.move_to_pole()
                break
            self.twist_pub.publish(twist_msg)

    def spin(self):
        # publish an empty goal so nothing happens with action client in the mean time
        # self.client.send_goal_and_wait(move_base_msgs.msg.MoveBaseGoal())
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 1
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1
        t = int(time.time())
        while int(time.time()) < t + 10:
            self.twist_pub.publish(twist_msg)

    def move_to_pole(self):
        print("IN FUNCTION")
        self.client.cancel_all_goals()
        h, w, d = self.cv_image.shape
        search_top = h / 4
        search_bot = 3 * h / 4 + 20
        self.mask[0:search_top, 0:w] = 0
        self.mask[search_bot:h, 0:w] = 0
        M = cv2.moments(self.mask)
        # print(M['m00'])
        if M['m00'] > 5000:  # If the area is greater than 0
            # find the x and y co-ordinates of the centroid of the region
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # get the pole at the center of the robot's vision
            error = cx - w / 2
            print(error)
            # Publish a twist command telling the robot to move to the pole
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = -float(error) / 100
            #s elf.twist_pub.publish(twist_msg)
            rospy.sleep(2)
            # try to send a move command to the pole
            # first find the depth
            depth = self.depth[cy, cx]
            if not math.isnan(depth):
                # now use the depth and trigonometry to find the x and y distance
                if self.orientation >= 0 and self.orientation <= math.pi / 2:
                    theta = self.orientation
                if self.orientation >= math.pi / 2:
                    theta = math.pi - self.orientation
                if self.orientation >= -math.pi and self.orientation <= -math.pi / 2:
                    theta = -math.pi - self.orientation
                if self.orientation >= -math.pi / 2 and self.orientation < 0:
                    theta = -math.pi / 2 - self.orientation
                xdistance = depth * math.cos(theta)
                ydistance = depth * math.sin(theta)
                print(self.orientation)
                print(depth)
                print(theta)
                # now check if the x or y distance needs to be negative
                if self.orientation >= 0 and self.orientation <= math.pi / 2:
                    xcor = round(self.posx + xdistance)
                    ycor = round(self.posy + ydistance)
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print(xcor, ycor)
                    self.move_client(xcor, ycor)
                if self.orientation >= math.pi / 2:
                    xcor = round(self.posx + -1 * xdistance)
                    ycor = round(self.posy + ydistance)
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print(xcor, ycor)
                    self.move_client(xcor, ycor)
                if self.orientation >= -math.pi and self.orientation <= -math.pi / 2:
                    xcor = round(self.posx + -1 * xdistance)
                    ycor = round(self.posy + -1 * ydistance)
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print(xcor, ycor)
                    self.move_client(xcor, ycor)
                if self.orientation >= -math.pi / 2 and self.orientation < 0:
                    xcor = round(self.posx + xdistance)
                    ycor = round(self.posy + -1 * ydistance)
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print(xcor, ycor)
                    self.move_client(xcor, ycor)
        # spin incase the robot didn't tag the pole
                self.spin()
        print("exiting function")

    def odom_cb(self, data):
        # save pos x and y in the class
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y
        # convert orientation to a euler angle and save it
        orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        (x, y, z) = euler_from_quaternion(orientation_list)
        # degrees = z * 180 / math.pi
        self.orientation = z

    def map_cb(self, data):
        self.map = np.array(data)
        print type(data)
        print self.map.shape


if __name__ == "__main__":
    rospy.init_node('move', anonymous=True)

    search = Search()
    # sleep to give callbacks a chance to initialise
    rospy.sleep(2)
    while search.colours_to_find:
        search.move_client(1, 4)
        search.spin_and_search()
        search.move_client(-1, 5)
        search.spin_and_search()
        search.move_client(1, -5)
        search.spin_and_search()
        search.move_client(-4, 0)
        search.spin_and_search()
        search.move_client(3, 0)
        search.spin_and_search()

    rospy.spin()
