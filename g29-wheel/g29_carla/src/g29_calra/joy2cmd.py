#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from carla_msgs.msg import CarlaEgoVehicleControl

cmd_pub = None
reverse = False

def joy_callback(data):
    global reverse
    # data.axes[0]     : steer              : -1 <= steer <= 1
    # data.axes[1]     : clutch             : -1 <= clutch <= 1
    # data.axes[2]     : throttle           : -1 <= throttle <= 1
    # data.axes[3]     : brake              : -1 <= brake <= 1
    # data.axes[4]     : cross-key          : left = 1, right = -1
    # data.axes[5]     : cross-key          : up = 1, down = -1
    # data.buttons[0]  : cross              : press = 1, else = 0
    # data.buttons[1]  : rect               : press = 1, else = 0
    # data.buttons[2]  : circle             : press = 1, else = 0
    # data.buttons[3]  : triangle           : press = 1, else = 0
    # data.buttons[4]  : R1(paddle shift +) : press = 1, else = 0
    # data.buttons[5]  : L1(paddle shift -) : press = 1, else = 0
    # data.buttons[6]  : R2                 : press = 1, else = 0
    # data.buttons[7]  : L2                 : press = 1, else = 0
    # data.buttons[8]  : shape              : press = 1, else = 0
    # data.buttons[9]  : options            : press = 1, else = 0
    # data.buttons[10] : R3                 : press = 1, else = 0
    # data.buttons[11] : L3                 : press = 1, else = 0
    # data.buttons[12] : 
    # data.buttons[13] : 
    # data.buttons[14] : 
    # data.buttons[15] : 
    # data.buttons[16] : 
    # data.buttons[17] : 
    # data.buttons[18] : 
    # data.buttons[19] : +                  : press = 1, else = 0
    # data.buttons[20] : -                  : press = 1, else = 0
    # data.buttons[21] : encoder c          : press = 1, else = 0
    # data.buttons[22] : encoder cc         : press = 1, else = 0
    # data.buttons[23] : enter              : press = 1, else = 0
    # data.buttons[24] : play station       : press = 1, else = 0
    cmd_msg = CarlaEgoVehicleControl()
    cmd_msg.throttle = (data.axes[2] + 1.0) * 0.5
    cmd_msg.brake = (data.axes[3] + 1.0) * 0.5
    cmd_msg.steer = data.axes[0]

    if data.buttons[19] == 1:
        reverse = False
    elif data.buttons[20] == 1:
        reverse = True
    cmd_msg.reverse = reverse

    cmd_pub.publish(cmd_msg)

def main():
    global cmd_pub

    rospy.init_node('carla_joy2cmd', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    manual_ctrl = rospy.get_param("~manual_ctrl", True)

    if manual_ctrl is True:
        topic_name = 'vehicle_control_cmd_manual'
    else:
        topic_name = 'vehicle_control_cmd'

    cmd_pub = rospy.Publisher("/carla/{}/{}".format(role_name, topic_name), CarlaEgoVehicleControl, queue_size=10)
    joy_sub = rospy.Subscriber("/g29/joy", Joy, joy_callback)

    rospy.spin()


if __name__ == "__main__":
    main()