# explorer.py
#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Explorer:
    def __init__(self):
        rospy.init_node('explorer')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/object_mapper/state', String, self.state_callback)
        
        # Parameters
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.exploration_time = rospy.get_param('~exploration_time', 2.0)
        
        # State
        self.mapper_state = 'IDLE'
        self.exploring = False
        self.rotation_accumulated = 0.0
        self.last_rotation_time = rospy.Time.now()
        self.full_rotation_threshold = 2 * math.pi
        
    def state_callback(self, msg):
        self.mapper_state = msg.data
        if self.mapper_state != 'IDLE':
            self.exploring = False
            
    def explore(self):
        """Basic exploration strategy"""
        if self.mapper_state != 'IDLE':
            return
            
        if not self.exploring:
            self.start_rotation()
        else:
            self.continue_rotation()
            
    def start_rotation(self):
        self.exploring = True
        self.rotation_accumulated = 0.0
        self.last_rotation_time = rospy.Time.now()
        
    def continue_rotation(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_rotation_time).to_sec()
        self.rotation_accumulated += self.angular_speed * dt
        self.last_rotation_time = current_time
        
        if self.rotation_accumulated >= self.full_rotation_threshold:
            self.move_forward()
        else:
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            
    def move_forward(self):
        """Move forward for a bit, then restart rotation"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(twist)
        
        rospy.Timer(rospy.Duration(self.exploration_time), 
                   lambda _: self.start_rotation(), 
                   oneshot=True)
        
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