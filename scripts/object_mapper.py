#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from ai3-roai-project.msg import DetectedObject 

class ObjectMapper:
    def __init__(self):
        rospy.init_node('object_mapper')
        
        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/object_detection', DetectedObject, self.object_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_pub = rospy.Publisher('/map_with_objects', OccupancyGrid, queue_size=10)
        self.state_pub = rospy.Publisher('/object_mapper/state', String, queue_size=10)
        
        # Parameters
        self.image_center = rospy.get_param('~image_center', 320)
        self.center_threshold = rospy.get_param('~center_threshold', 10)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.min_object_distance = {
            'chair': rospy.get_param('~min_chair_distance', 0.5),
            'table': rospy.get_param('~min_table_distance', 1.0)
        }
        self.object_sizes = {
            'chair': rospy.get_param('~chair_size', 2),  # Radius in cells
            'table': rospy.get_param('~table_size', 4)   # Radius in cells
        }
        
        # Store data
        self.latest_scan = None
        self.current_map = None
        self.mapped_objects = {
            'chair': [],  # List of (x, y) tuples
            'table': []   # List of (x, y) tuples
        }
        
        # State variables
        self.state = 'IDLE'  # States: IDLE, CENTERING, MAPPING
        self.is_rotating = False
        self.current_object_type = None
        
        rospy.loginfo("Object mapper initialized")

    def laser_callback(self, scan_msg):
        """Store the latest laser scan"""
        self.latest_scan = scan_msg

    def map_callback(self, map_msg):
        """Store the latest map"""
        self.current_map = map_msg

    def object_callback(self, obj_msg):
        """Handle incoming object detections"""
        if self.latest_scan is None or self.current_map is None:
            return

        if obj_msg.object_type not in ['chair', 'table']:
            rospy.logwarn(f"Unknown object type: {obj_msg.object_type}")
            return

        self.current_object_type = obj_msg.object_type
        self.state = 'CENTERING'
        self.publish_state()
        self.center_on_object(obj_msg.x)

    def publish_state(self):
        """Publish current state"""
        self.state_pub.publish(String(self.state))

    def center_on_object(self, obj_img_x):
        """Center the robot on the detected object"""
        error = obj_img_x - self.image_center
        
        if abs(error) > self.center_threshold:
            self.rotate_to_object(error)
        else:
            self.stop_rotation()
            self.state = 'MAPPING'
            self.process_centered_object()

    def rotate_to_object(self, error):
        """Rotate the robot to center the object"""
        twist = Twist()
        twist.angular.z = -self.angular_speed if error > 0 else self.angular_speed
        self.cmd_vel_pub.publish(twist)
        self.is_rotating = True

    def stop_rotation(self):
        """Stop the robot's rotation"""
        if self.is_rotating:
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.is_rotating = False

    def process_centered_object(self):
        """Process and map the centered object"""
        if self.latest_scan is None or self.current_object_type is None:
            return

        middle_index = len(self.latest_scan.ranges) // 2
        object_distance = self.latest_scan.ranges[middle_index]
        
        if not math.isfinite(object_distance):
            rospy.logwarn("Invalid distance reading")
            self.state = 'IDLE'
            self.publish_state()
            return

        object_angle = self.latest_scan.angle_min + (middle_index * self.latest_scan.angle_increment)
        object_position = self.laser_to_map_coordinates(object_angle, object_distance)

        if not self.is_object_already_mapped(object_position, self.current_object_type):
            self.mark_object_on_map(object_position, self.current_object_type)
            self.mapped_objects[self.current_object_type].append(object_position)
            rospy.loginfo(f"New {self.current_object_type} mapped at position: ({object_position[0]:.2f}, {object_position[1]:.2f})")
        else:
            rospy.loginfo(f"{self.current_object_type} already mapped at this position")

        self.state = 'IDLE'
        self.current_object_type = None
        self.publish_state()

    def laser_to_map_coordinates(self, angle, distance):
        """Convert laser polar coordinates to map coordinates"""
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return (x, y)

    def is_object_already_mapped(self, new_position, object_type):
        """Check if an object has already been mapped at this position"""
        min_distance = self.min_object_distance[object_type]
        for pos in self.mapped_objects[object_type]:
            distance = math.sqrt((new_position[0] - pos[0])**2 + 
                               (new_position[1] - pos[1])**2)
            if distance < min_distance:
                return True
        return False

    def mark_object_on_map(self, position, object_type):
        """Mark the object position on the map"""
        if self.current_map is None:
            return

        new_map = OccupancyGrid()
        new_map.header = self.current_map.header
        new_map.info = self.current_map.info
        new_map.data = list(self.current_map.data)

        cell_x = int((position[0] - new_map.info.origin.position.x) / 
                    new_map.info.resolution)
        cell_y = int((position[1] - new_map.info.origin.position.y) / 
                    new_map.info.resolution)

        # Get object size in cells
        size = self.object_sizes[object_type]
        
        # Mark the object on the map
        for dx in range(-size, size + 1):
            for dy in range(-size, size + 1):
                # For tables, make a rectangular shape
                if object_type == 'table' and abs(dx) == size and abs(dy) == size:
                    continue  # Skip corners for tables
                self.mark_cell(new_map, cell_x + dx, cell_y + dy)

        self.map_pub.publish(new_map)

    def mark_cell(self, map_msg, cell_x, cell_y):
        """Mark a single cell in the occupancy grid"""
        if (0 <= cell_x < map_msg.info.width and 
            0 <= cell_y < map_msg.info.height):
            index = cell_y * map_msg.info.width + cell_x
            map_msg.data[index] = 100  # Mark as occupied

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        mapper = ObjectMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass