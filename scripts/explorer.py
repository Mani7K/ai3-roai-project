#!/usr/bin/env python
import rospy
import math
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
        self.angular_speed = rospy.get_param('~angular_speed', 0.1)
        self.min_distance = rospy.get_param('~min_distance', 0.5)
        
        # State
        self.mapper_state = 'IDLE'
        self.latest_scan = None
        self.current_direction = 1  # 1 for CCW, -1 for CW
        
    def state_callback(self, msg):
        self.mapper_state = msg.data
        
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def get_sector_distances(self):
        """Get minimum distances in different sectors around the robot"""
        if not self.latest_scan:
            return None
            
        ranges = self.latest_scan.ranges
        size = len(ranges)
        
        # Define sectors (front, front-left, front-right)
        sectors = {
            'front': ranges[size//2-20:size//2+20],
            'front_left': ranges[size//4-20:size//4+20],
            'front_right': ranges[3*size//4-20:3*size//4+20]
        }
        
        # Get minimum valid distance for each sector
        min_distances = {}
        for sector, measurements in sectors.items():
            valid_ranges = [r for r in measurements if r != float('inf')]
            min_distances[sector] = min(valid_ranges) if valid_ranges else float('inf')
            
        return min_distances

    def explore(self):
        """Circular exploration with wall avoidance"""
        if self.mapper_state != 'IDLE':
            self.cmd_vel_pub.publish(Twist())
            return
            
        distances = self.get_sector_distances()
        if not distances:
            self.cmd_vel_pub.publish(Twist())
            return
            
        twist = Twist()
        
        # Base circular movement
        twist.angular.z = self.angular_speed * self.current_direction
        twist.linear.x = self.linear_speed
        
        # Wall avoidance logic
        front_close = distances['front'] < self.min_distance
        left_close = distances['front_left'] < self.min_distance
        right_close = distances['front_right'] < self.min_distance
        
        if front_close:
            # If wall directly ahead, reduce forward speed and turn more
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = self.angular_speed * 1.5 * self.current_direction
            rospy.loginfo(f"Wall ahead at {distances['front']}m, turning sharper")
            
        elif left_close and self.current_direction > 0:
            # If wall on left and turning CCW, switch direction
            self.current_direction = -1
            twist.angular.z = self.angular_speed * self.current_direction
            rospy.loginfo("Wall on left, switching to CW")
            
        elif right_close and self.current_direction < 0:
            # If wall on right and turning CW, switch direction
            self.current_direction = 1
            twist.angular.z = self.angular_speed * self.current_direction
            rospy.loginfo("Wall on right, switching to CCW")
            
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.explore()
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = Explorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass