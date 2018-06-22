#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 

def twist_cb(data):
    pass

def twist_listener():
    rospy.init_node('twist_listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, twist_cb)


class DiffDrive(object):
    
    def __init__(self):
        print("creando pubs...")
        self.left_motor_pub = rospy.Publisher('wheel_power_left', Float32, queue_size=5)
        self.right_motor_pub = rospy.Publisher('wheel_power_right', Float32, queue_size=5)
        print("creando sub...")
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.twist_cb)
        rospy.init_node('diff_drive', anonymous=True)
    
    def twist_cb(self, data):
        linear = data.linear.x
        angular = data.angular.z
        # diff drive conversion
        K = 0.6
        v_r = K * linear + (1 - K) * angular
        v_l = K * linear - (1 - K) * angular
        left_msg = Float32()
        left_msg.data = v_l
        right_msg = Float32()
        right_msg.data = v_r
        

        self.left_motor_pub.publish(left_msg)
        self.right_motor_pub.publish(right_msg)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    d = DiffDrive()
    try:
        d.run()
    except:
        print("error!")