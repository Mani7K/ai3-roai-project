#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class Explorer:
    def __init__(self):
        rospy.init_node('explorer')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/object_mapper/state', String, self.state_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.min_front_distance = rospy.get_param('~min_front_distance', 0.5)
        
        # State
        self.mapper_state = 'IDLE'
        self.latest_scan = None
        
    def state_callback(self, msg):
        self.mapper_state = msg.data
        
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def get_front_distance(self):
        """Get the minimum distance in front of the robot"""
        if not self.latest_scan:
            return None
            
        # Get the middle portion of the scan (front of robot)
        ranges = self.latest_scan.ranges
        center_index = len(ranges) // 2
        front_ranges = ranges[center_index-20:center_index+20]  # Look at 40 degrees arc in front
        
        # Filter out invalid readings
        valid_ranges = [r for r in front_ranges if r != float('inf')]
        
        if valid_ranges:
            return min(valid_ranges)
        return None

    def explore(self):
        """Simple exploration: move forward, turn left when obstacle detected"""
        if self.mapper_state != 'IDLE':
            # Stop if mapper is active
            self.cmd_vel_pub.publish(Twist())
            return
            
        front_distance = self.get_front_distance()
        if front_distance is None:
            # No valid reading, stop for safety
            self.cmd_vel_pub.publish(Twist())
            return
            
        twist = Twist()
        
        if front_distance < self.min_front_distance:
            # Wall ahead, turn left
            rospy.loginfo(f"Wall detected at {front_distance}m, turning left")
            twist.angular.z = self.angular_speed  # Positive for left turn
        else:
            # No wall, move forward
            twist.linear.x = self.linear_speed
            
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.explore()
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass