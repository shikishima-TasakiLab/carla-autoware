#!/usr/bin/env python
from carla_msgs.msg import CarlaEgoVehicleStatus
from g29_force_feedback.msg import ForceFeedback
from std_msgs.msg import Bool
import rospy

feedback_pub = rospy.Publisher("/feedback", ForceFeedback, queue_size=10)
manual = False

def vehicle_status_callback(data):
    feedback_msg = ForceFeedback()
    if manual is True:
        feedback_msg.angle = 0.0
        feedback_msg.force = 0.2
        feedback_msg.pid_mode = False
    else:
        feedback_msg.angle = data.control.steer
        feedback_msg.force = 0.2
        feedback_msg.pid_mode = True
    feedback_pub.publish(feedback_msg)

def vehicle_control_manual_override_callback(data):
    global manual
    manual = data.data

def main():
    rospy.init_node('carla_feedback', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    print role_name

    vehicle_control_manual_override_sub = rospy.Subscriber("/carla/{}/vehicle_control_manual_override".format(role_name), Bool, vehicle_control_manual_override_callback)
    vehicle_status_sub = rospy.Subscriber("/carla/{}/vehicle_status".format(role_name), CarlaEgoVehicleStatus, vehicle_status_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
