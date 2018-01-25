#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import sys

# Detection is considered valid after 'n' repeated observations
STATE_COUNT_THRESHOLD = 3

class Pose_positon:
    def __init__(self, t):
        self.x = t[0]
        self.y = t[1]

class TLDetector(object):
    def __init__(self):
        # Start 'seeing' traffic lights only when we're some meters away!
        # - faster_rcnn starts seeing around 60m away
        self.MIN_DETECTION_DIST = 80.0

        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.lights = []
        self.tl_wps = []
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.tl_positions = [Pose(position=Point(x= p[0], y= p[1])) for p in self.config['stop_line_positions']]

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        #print(self.config)
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        # image size        
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        scenes = self.config['camera_info']['scenes']
        #self.light_classifier = TLClassifier(fx,fy,image_width,image_height,scenes)
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.camera_image = None

        # Publishes the state for each traffic light, useful to get training data
        self.TrafficLightState = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.lights = []

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def euclidean_distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
<<<<<<< HEAD
        """
        min_dist = sys.maxint
        if self.waypoints is not None:
            for i, point in enumerate(self.waypoints.waypoints):
                dist = self.euclidean_distance(pose.position, point.pose.pose.position)
                if dist < min_dist:
                    closest_idx = i
                    min_dist = dist
            return closest_idx
=======

        """
        #TODO implement
        closest_index = -1
        closest_dis = -1

        if self.waypoints is not None:
            waypoints = self.waypoints.waypoints
            for i in range(len(waypoints)):
                #print(pose)
                dis = (waypoints[i].pose.pose.position.x - pose.x) ** 2 + (waypoints[i].pose.pose.position.y - pose.y) ** 2

                if (closest_dis == -1) or (closest_dis > dis):
                    closest_dis = dis
                    closest_index = i
        return closest_index


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        print("got a img")
        #cv2.imshow("test",cv_image)
        #k = cv2.waitKey(1)
        #Get classification
        return self.light_classifier.get_classification(cv_image)
>>>>>>> origin/cv_camera

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
<<<<<<< HEAD

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # Find closest stopping position to current pose
        closest_tl_dist = sys.maxint
        closest_tl_idx = None
        for i in range(len(self.tl_positions)):
            dist = self.euclidean_distance(self.tl_positions[i].position, self.pose.pose.position)
            if dist < closest_tl_dist:
                closest_tl_dist = dist
                closest_tl_idx = i

        # rospy.loginfo('Closest tl: {},  distance={}'.format(closest_tl_idx, closest_tl_dist))

        if (closest_tl_dist >= self.MIN_DETECTION_DIST):
            return -1, TrafficLight.UNKNOWN
        else:
            classification = self.light_classifier.get_classification(cv_image)
            # rospy.loginfo('Image classification={}'.format(classification))

            closest_tl_waypoint_idx = self.get_closest_waypoint(self.tl_positions[closest_tl_idx])

            return closest_tl_waypoint_idx, classification
=======
        light = None
        if (self.waypoints is None):
            return (-1, TrafficLight.UNKNOWN)

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if (len(self.tl_wps)==0):           
            for i, stop_line in enumerate(stop_line_positions):
                tl_wp = self.get_closest_waypoint(Pose_positon(stop_line))
                self.tl_wps.append( (tl_wp+5)) 

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)
            minDist = self.track_index_diff(car_position, self.tl_wps[0])
            light_index = 0
            for i in range(1, len(self.tl_wps)):
                dist = self.track_index_diff(car_position, self.tl_wps[i])
                if (dist < minDist):
                    minDist = dist                    
                    light_index = i
            
            light = self.lights[light_index]
            print("light: ", light_index, minDist)
            
        #find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return minDist, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN
>>>>>>> origin/cv_camera

    def track_index_diff(self,index1, index2):
        if (self.waypoints.waypoints is None):
            return -1        
        N = len(self.waypoints.waypoints)
        if index2 >= index1 :
            return index2 - index1
        else:
            return N - index1 - 1 + index2


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
