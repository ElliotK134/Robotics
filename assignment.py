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


class Search:

    def __init__(self):
        # publishers
        # this node will contain all of the subscribers and publishers
        # this is the actionlib client which will publish move_base commands
        self.client = simple_action_client.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
        # this will publish twist commands to make the robot spin or stop
        self.twist_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        # open cv functionality
        cv2.startWindowThread()
        self.bridge = CvBridge()
        # colour image subscriber
        self.image_colour = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        # depth image subscriber
        self.image_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.colours_to_find = ['red', 'green', 'yellow', 'blue']
        # move_base goal message type
        self.goal = move_base_msgs.msg.MoveBaseGoal()
        # odometry subscriber
        self.odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        # positions stored here from odometry
        self.orientation = 0
        self.x = 0
        self.y = 0

    def move_client(self, x, y):
        self.x = x
        self.y = y
        # wait for the server before sending any goals
        self.client.wait_for_server()
        # define a move_base goal with the x and y coordinates
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1
        # publish the goal
        print("goal published")
        # send the goal and wait for it to finish
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
            # self.move_client(newx, newy)
            # spin to look for pole
            # print("sent new goal to", newx, newy)
            # self.spin()
            # print("exiting done_cb")

    def callback(self, data):
        cv2.namedWindow("Segmentation", 1)
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # convert the image to hsv
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # this block defines upper and lower bounds for each colour in hsv, then constructs a mask containing all of them

        # check red hasn't been found
        if 'red' in self.colours_to_find:
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([5, 360, 360])
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            self.mask = red_mask

        # check blue hasn't been found
        if 'blue' in self.colours_to_find:
            lower_blue = np.array([100, 120, 100])
            upper_blue = np.array([120, 360, 360])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            # if a mask already exists, add blue to it
            if 'red' in self.colours_to_find:
                self.mask = self.mask + blue_mask
            else:
                # else the mask becomes just the blue mask
                self.mask = blue_mask

        # check yellow hasn't been found
        if 'yellow' in self.colours_to_find:
            lower_yellow = np.array([20, 120, 100])
            upper_yellow = np.array([30, 360, 360])
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            # if a mask already exists, add yellow to it
            if 'red' in self.colours_to_find or 'blue' in self.colours_to_find:
                self.mask = self.mask + yellow_mask
            else:
                # else the mask becomes the yellow mask
                self.mask = yellow_mask

        # check green hasn't been found
        if 'green' in self.colours_to_find:
            lower_green = np.array([50, 120, 100])
            upper_green = np.array([65, 360, 360])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            # if a mask already exists, add green to it
            if 'red' in self.colours_to_find or 'blue' in self.colours_to_find or 'yellow' in self.colours_to_find:
                self.mask = self.mask + green_mask
            else:
                # else the mask becomes just the green mask
                self.mask = green_mask

        # now apply a mask to segment the poles
        self.threshImg = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.mask)
        cv2.imshow("Segmentation", self.threshImg)
        cv2.waitKey(1)

        # this block will check if the robot is in one meter of a pole using the depth camera
        # if it is, it will find which pole it is near and then print that the pole has been found

        # first get the shape of the image
        # the important values are height and width
        h, w, d = self.cv_image.shape
        # search the bottom of the image
        search_region = h / 2
        self.mask[0:search_region, 0:w] = 0
        M = cv2.moments(self.mask)
        if M['m00'] > 0:  # If the area is greater than 0
            # find the x and y co-ordinates of the centroid of the region
            # These two lines to find a centroid are from the opencv documentation
            # https://docs.opencv.org/trunk/dd/d49/tutorial_py_contour_features.html
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # now use the depth image to see how far the robot is from the object at the centroid calculated earlier
            # if the robot is under 1 meter away, stop
            depth = self.depth[cy, cx]
            if depth < 1.0:
                print("depth =", depth)
                # define a twist message which will make the robot stop
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                # get the colour of the object found
                print(self.threshImg[cy, cx][0:3])
                # pixel range for yellow
                if np.all(self.threshImg[cy, cx] >= [0, 90, 90]) and np.all(self.threshImg[cy, cx] < [10, 200, 200]):
                    print("found yellow")
                    # remove yellow from colours_to_find
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'yellow']
                    print(self.colours_to_find)
                    rospy.sleep(1)
                # pixel range for green
                elif np.all(self.threshImg[cy, cx] >= [0, 90, 0]) and np.all(self.threshImg[cy, cx] < [10, 200, 10]):
                    print("found green")
                    # remove green from colours_to_find
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'green']
                    print(self.colours_to_find)
                    rospy.sleep(1)
                # pixel range for red
                elif np.all(self.threshImg[cy, cx] >= [0, 0, 90]) and np.all(self.threshImg[cy, cx] < [10, 10, 200]):
                    print("found red")
                    # remove red from colours_to_find
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'red']
                    print(self.colours_to_find)
                    rospy.sleep(1)
                # pixel range for blue
                elif np.all(self.threshImg[cy, cx] >= [90, 0, 0]) and np.all(self.threshImg[cy, cx] < [200, 10, 10]):
                    print("found blue")
                    # remove blue from colours_to_find
                    self.colours_to_find = [i for i in self.colours_to_find if i != 'blue']
                    print(self.colours_to_find)
                    rospy.sleep(1)
                # publish the twist message to stop infront of the robot
                self.twist_pub.publish(twist_msg)
                rospy.sleep(3)

    def depth_image_callback(self, data):
        # processes the depth image
        self.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def spin_and_search(self):
        # cancel all goals so move commands don't conflict
        self.client.cancel_all_goals()
        # define a twist command to spin
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 1
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1
        # get the time now
        t = int(time.time())
        print("spinning")
        # spin for 10 seconds
        while int(time.time()) < t + 10:
            h, w, d = self.cv_image.shape
            # search the bottom of the image
            search_region = h / 2
            self.mask[0:search_region, 0:w] = 0
            M = cv2.moments(self.mask)
            if M['m00'] > 5000:  # If the area is greater than 0
                # get the centroid of the area and stop
                twist_msg.angular.z = 0
                self.twist_pub.publish(twist_msg)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # try to send a move command to the pole
                # first find the depth
                depth = self.depth[cy, cx]
                if not math.isnan(depth):
                    # proceed only if the depth is clear
                    self.move_to_pole()
                break
            self.twist_pub.publish(twist_msg)

    def spin(self):
        # define a twist message to spin on the spot
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 1
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1
        t = int(time.time())
        # get the time now and use it to spin for 10 seconds
        while int(time.time()) < t + 10:
            self.twist_pub.publish(twist_msg)

    def move_to_pole(self):
        print("IN FUNCTION")
        # cancel all goals so move commands don't conflict
        self.client.cancel_all_goals()
        # search the bottom of the image
        h, w, d = self.cv_image.shape
        search_region = h / 2
        self.mask[0:search_region, 0:w] = 0
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
            # if error is negative, turn right
            t = int(time.time())
            if error < 0:
                twist_msg.angular.z = 0.5
                print("turning anticlockwise")
                while error < 0:
                    # recalculate the error and spin if necessary
                    M = cv2.moments(self.mask)
                    if M['m00'] > 0:  # this prevents divide by 0 errors
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                    error = cx - w / 2
                    self.twist_pub.publish(twist_msg)
                    if int(time.time()) > t + 10:  # stops the loop from becoming infinite if the object disappears
                        break

            else:
                # else turn left
                twist_msg.angular.z = -0.5
                print("turning clockwise")
                while error > 0:
                    # recalculate the error and spin if necessary
                    M = cv2.moments(self.mask)
                    if M['m00'] > 0:  # this prevents divide by 0 errors
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                    error = cx - w / 2
                    self.twist_pub.publish(twist_msg)
                    if int(time.time()) > t + 10:  # stops the loop from becoming infinite if the object disappears
                        break

            # stop when the pole is at the center of the vision
            twist_msg.angular.z = 0
            self.twist_pub.publish(twist_msg)
            rospy.sleep(2)
            # try to send a move command to the pole
            # first find the depth
            depth = self.depth[cy, cx]
            if not math.isnan(depth):  # check again if the depth is defined
                # now use the depth and trigonometry to find the x and y distance
                # for facing top right
                if self.orientation >= 0 and self.orientation <= math.pi / 2:
                    theta = self.orientation
                # for facing top left
                if self.orientation >= math.pi / 2:
                    theta = math.pi - self.orientation
                # for facing bottom left
                if self.orientation >= -math.pi and self.orientation <= -math.pi / 2:
                    theta = -math.pi + self.orientation
                # for facing bottom right
                if self.orientation >= -math.pi / 2 and self.orientation < 0:
                    theta = 0 - self.orientation
                # calculate the x and y distance
                xdistance = depth * math.cos(theta)
                ydistance = depth * math.sin(theta)
                print(self.orientation)
                print(depth)
                print(theta)
                # now check if the x or y distance needs to be negative
                # if facing up right
                if self.orientation >= 0 and self.orientation <= math.pi / 2:
                    # neither distance should be negative
                    xcor = round(self.posx + xdistance)
                    ycor = round(self.posy + ydistance)
                    # make sure the command isn't going out of bounds
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print("facing up right", xcor, ycor)
                    # send the move command
                    self.move_client(xcor, ycor)
                # if facing up left
                elif self.orientation >= math.pi / 2:
                    # the x distance is negative
                    xcor = round(self.posx + -1 * xdistance)
                    ycor = round(self.posy + ydistance)
                    # make sure the command isn't going out of bounds
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print("facing up left", xcor, ycor)
                    # send the command
                    self.move_client(xcor, ycor)
                elif self.orientation >= -math.pi and self.orientation <= -math.pi / 2:
                    # both x and y distances are negative
                    xcor = round(self.posx + -1 * xdistance)
                    ycor = round(self.posy + -1 * ydistance)
                    # make sure the command isn't going out of bounds
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print("facing down left", xcor, ycor)
                    # send the command
                    self.move_client(xcor, ycor)
                elif self.orientation >= -math.pi / 2 and self.orientation < 0:
                    # the y distance is negative
                    xcor = round(self.posx + xdistance)
                    ycor = round(self.posy + -1 * ydistance)
                    # make sure the command isn't going out of bounds
                    if xcor > 4:
                        xcor = 4
                    if ycor > 5:
                        ycor = 5
                    if xcor < -4:
                        xcor = -4
                    if ycor < -5:
                        ycor = -5
                    print("facing down right", xcor, ycor)
                    # send the command
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
        self.orientation = z


if __name__ == "__main__":
    rospy.init_node('move', anonymous=True)

    search = Search()
    # sleep to give callbacks a chance to initialise
    rospy.sleep(2)
    while len(search.colours_to_find) != 0:
        search.move_client(2, 4)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(-1, 3)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(3, 1)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(0, -1)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(-4, 0)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(-4, 2)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(1, -3)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(-3, -5)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(1, -4)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break
        search.move_client(-4, 4.5)
        search.spin_and_search()
        if len(search.colours_to_find) == 0:
            break


    print("Found all objects!")
    rospy.spin()
    cv2.destroyAllWindows()
