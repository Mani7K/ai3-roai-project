#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math

class Explorer:
    def __init__(self):
        rospy.init_node('explorer')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        
        self.min_distance = 2.0
        self.state = 'explore'
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.current_map = None
        self.map_resolution = 0
        self.map_origin = None

    def map_callback(self, msg):
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

    def find_unknown_direction(self):
        if self.current_map is None:
            return None
        
        # Get robot position in map coordinates
        robot_x = int((-self.map_origin.position.x) / self.map_resolution)
        robot_y = int((-self.map_origin.position.y) / self.map_resolution)
        
        # Search for unknown cells (-1 in occupancy grid)
        unknown = np.where(self.current_map == -1)
        if len(unknown[0]) == 0:
            return None
            
        # Find closest unknown cell
        distances = np.sqrt((unknown[1] - robot_x)**2 + (unknown[0] - robot_y)**2)
        closest_idx = np.argmin(distances)
        
        # Calculate angle to target
        target_y = unknown[0][closest_idx] - robot_y
        target_x = unknown[1][closest_idx] - robot_x
        return math.atan2(target_y, target_x)

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[ranges == float('inf')] = scan.range_max
        
        front_angles = len(ranges) // 6
        front_scan = ranges[-front_angles//2:].tolist() + ranges[:front_angles//2].tolist()
        min_front_dist = min(front_scan)
        
        target_direction = self.find_unknown_direction()
        
        if min_front_dist < self.min_distance:
            # Obstacle avoidance
            self.state = 'avoiding'
            sectors = np.array_split(ranges, 8)
            avg_distances = [np.mean(sector) for sector in sectors]
            best_sector = np.argmax(avg_distances)
            
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5 if best_sector < 4 else -0.5
        
        elif target_direction is not None:
            # Move towards unknown area
            self.state = 'explore'
            angle_diff = target_direction
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = np.clip(angle_diff, -0.5, 0.5)
        
        else:
            # Default movement
            self.state = 'forward'
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

    def run(self):
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass