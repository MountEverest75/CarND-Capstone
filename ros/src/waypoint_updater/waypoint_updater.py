#!/usr/bin/env python

import rospy
import copy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from enum import Enum

import math
import sys
import tf
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 200
PUBLISH_RATE = 20
# Target velocity will be set to 0 for each waypoint within this distance from the target waypoint
STOP_DISTANCE = 3

class State(Enum):
    DRIVE = 0
    STOP = 1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        # Acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.
        self.accel_limit = rospy.get_param('~accel_limit', 1.)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add State variables
        self.state = State.DRIVE
        self.base_waypoints = []  # List of waypoints, as received from /base_waypoints
        self.current_pose = None # Car pose
        self.stop_waypoint = None # Waypoint index of the next red light
        self.msg_seq = 0 # Sequence number of /final_waypoints message

        # Start periodic publishing into /final_waypoints
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()


    def update_and_publish(self):
        next_waypoint_index = self._find_next_waypoint()

        if next_waypoint_index is not None:
            # rospy.logwarn('Next waypoint: {} v: {} av: {}'.format(next_waypoint_index, self.base_waypoints[next_waypoint_index].twist.twist.linear,  self.base_waypoints[next_waypoint_index].twist.twist.angular))

            num_base_wp = len(self.base_waypoints)
            waypoint_indices = [idx % num_base_wp for idx in range( next_waypoint_index,  next_waypoint_index+LOOKAHEAD_WPS)]
            final_waypoints = [self.base_waypoints[wp] for wp in waypoint_indices]

            if self.state == State.STOP \
                    and self.stop_waypoint \
                    and self.stop_waypoint in waypoint_indices:
                stop_wp_index = waypoint_indices.index(self.stop_waypoint)
                final_waypoints = self._decelerate(final_waypoints, next_waypoint_index, stop_wp_index)

            self.publish_msg(final_waypoints)
        # elif self.base_waypoints:
            # rospy.logerr("Couldn't find next navigation waypoint - lost tracking?")

    def _decelerate(self, waypoints, next_waypoint_idx, stop_index):
        """
        Decelerate a list of wayponts so that they stop on stop_index
        """
        if stop_index <= 0:
            return waypoints

        new_waypoints = copy.deepcopy(waypoints)

        dist = self.distance(new_waypoints, 0, stop_index)
        step = dist / stop_index

        d = 0.
        for idx in reversed(range(stop_index)):
            v = -1.

            d += step
            # Set speed to 0 around the stopping waypoint
            if d > STOP_DISTANCE:
                v = math.sqrt(2 * abs(self.accel_limit) * d )

                if v < 1.:
                    v = 0.
            self.set_waypoint_velocity(new_waypoints, idx, v)

        return new_waypoints

    def _find_next_waypoint(self):
        # Safeguard for initialization state
        if not self.current_pose or not self.base_waypoints:
            return

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        closest_waypoint_index = None
        dist = sys.maxint

        # Find closest waypoint
        n_waypoints = len(self.base_waypoints)
        for i in range(n_waypoints):
            wp_d = dl(self.current_pose.position, self.base_waypoints[i].pose.pose.position)

            if wp_d < dist:
                dist = wp_d
                closest_waypoint_index = i

        map_x = self.base_waypoints[closest_waypoint_index].pose.pose.position.x
        map_y = self.base_waypoints[closest_waypoint_index].pose.pose.position.y

        heading = math.atan2((map_y - self.current_pose.position.y), (map_x - self.current_pose.position.x))
        quaternion = (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)

        if angle > (math.pi / 4):
            closest_waypoint_index += 1

        return closest_waypoint_index

    def publish_msg(self, final_waypoints):
        waypoint_msg = Lane()
        waypoint_msg.header.seq = self.msg_seq
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = '/world'
        waypoint_msg.waypoints = final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)
        self.msg_seq += 1

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        waypoints = msg.waypoints
        num_wp = len(waypoints)

        self.base_waypoints = waypoints


    def traffic_cb(self, msg):
        # rospy.loginfo("Got traffic light waypoint: {}".format(msg.data))
        if msg.data:
            self.stop_waypoint = msg.data
            self.state = State.STOP
        else:
            self.stop_waypoint = None
            self.state = State.DRIVE

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """ Calculates total distance between the waypoints starting in index wp1 and ending in wp2+1  """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
