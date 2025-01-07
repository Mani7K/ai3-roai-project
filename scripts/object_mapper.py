#!/usr/bin/env python

import rospy
import math
import json
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class ObjectMapper:
    def __init__(self):
        rospy.init_node('object_mapper')
        
        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/object_detection', String, self.object_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=10)
        self.state_pub = rospy.Publisher('/object_mapper/state', String, queue_size=10)
        
        # Parameters
        self.image_center = rospy.get_param('~image_center', 320)
        self.center_threshold = rospy.get_param('~center_threshold', 25)
        self.angular_speed = rospy.get_param('~angular_speed', 0.05)
        self.min_confidence = rospy.get_param('~min_confidence', 0.6)
        
        # Object properties
        self.valid_classes = ['chair', 'diningtable']
        self.object_colors = {
            'chair': ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            'diningtable': ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue
        }
        self.object_sizes = {
            'chair': {'type': Marker.CYLINDER, 'scale': (0.4, 0.4, 0.5)},
            'diningtable': {'type': Marker.CUBE, 'scale': (0.8, 0.8, 0.4)}
        }
        
        # State variables
        self.latest_scan = None
        self.current_map = None
        self.mapped_objects = {class_name: [] for class_name in self.valid_classes}
        self.state = 'IDLE'
        self.target_object = None
        self.marker_id = 0
        
        rospy.loginfo("Object mapper initialized")

    def laser_callback(self, scan_msg):
        self.latest_scan = scan_msg

        if self.state == 'WAITING_FOR_SCAN':
            self.process_centered_object()

    def map_callback(self, map_msg):
        self.current_map = map_msg

    def object_callback(self, obj_msg):
        if self.latest_scan is None or self.current_map is None:
            return

        try:
            detections = json.loads(obj_msg.data)
            valid_detections = [
                obj for obj in detections
                if obj.get("class") in self.valid_classes and 
                   obj.get("confidence", 0) >= self.min_confidence
            ]

            if valid_detections and self.state == 'IDLE':
                # Select closest object to center
                self.target_object = min(
                    valid_detections,
                    key=lambda obj: abs(obj['bbox'][0] - self.image_center)
                )
                self.state = 'CENTERING'
                self.publish_state()
                rospy.loginfo(f"New target: {self.target_object['class']}")
            
            if self.state == 'CENTERING':
                if not valid_detections:
                    self.state = 'IDLE'
                else:
                    self.target_object = min(
                        valid_detections,
                        key=lambda obj: abs(obj['bbox'][0] - self.image_center)
                    )
                    self.center_on_object()

        except json.JSONDecodeError:
            rospy.logwarn("Invalid JSON from detector")

    def center_on_object(self):
        rospy.loginfo("Centering on object")
        error = self.target_object['bbox'][0] - self.image_center
        rospy.loginfo(self.target_object['bbox'])
        
        if abs(error) > self.center_threshold:
            rospy.loginfo("Target not centered")
            twist = Twist()
            rospy.loginfo(f"Error: {error}")
            twist.angular.z = self.angular_speed if error < 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            
        else:
            rospy.loginfo("Target centered")
            # Stop and process the centered object
            self.state = 'WAITING_FOR_SCAN'
            self.cmd_vel_pub.publish(Twist())

    def process_centered_object(self):
        if not self.latest_scan:
            return

        middle_index = len(self.latest_scan.ranges) // 2
        distance = self.latest_scan.ranges[middle_index]
        
        if not math.isfinite(distance):
            rospy.logwarn("Invalid distance reading")
            self.finish_mapping()
            return

        angle = self.latest_scan.angle_min + (middle_index * self.latest_scan.angle_increment)
        angle = -angle  # Correct for the orientation of the robot
        position = (
            distance * math.cos(angle),
            distance * math.sin(angle)
        )

        object_type = self.target_object['class']
        if not any(math.hypot(position[0] - p[0], position[1] - p[1]) < 0.5 
                  for p in self.mapped_objects[object_type]):
            self.publish_marker(position, object_type)
            self.mapped_objects[object_type].append(position)
            rospy.loginfo(f"Mapped {object_type} at ({position[0]:.2f}, {position[1]:.2f})")
        
        self.finish_mapping()

    def finish_mapping(self):
        self.target_object = None
        self.state = 'IDLE'
        self.publish_state()

    def publish_marker(self, position, object_type):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = object_type
        marker.id = self.marker_id
        self.marker_id += 1

        props = self.object_sizes[object_type]
        marker.type = props['type']
        marker.scale.x, marker.scale.y, marker.scale.z = props['scale']
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = props['scale'][2]/2
        marker.pose.orientation.w = 1.0
        
        marker.color = self.object_colors[object_type]
        marker.lifetime = rospy.Duration(0)
        
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_state(self):
        self.state_pub.publish(String(self.state))

    def run(self):
        # rospy.spin()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        mapper = ObjectMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass