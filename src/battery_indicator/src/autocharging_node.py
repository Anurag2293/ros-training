#!/usr/bin/env python3
import rospy
from battery_indicator.msg import ErrorStatus, BatteryStatus
from std_srvs.srv import SetBool, SetBoolRequest

class AutoChargingNode:
    def __init__(self):
        rospy.init_node('autocharging_node')

        # ROS parameters (default values provided, can be overridden in the launch file)
        self.critical_percent = rospy.get_param('~critical_percent', 15)
        self.full_battery = rospy.get_param('~full_battery', 90)
        self.warning_percentage = rospy.get_param('~warning_percentage', 25)

        # Subscriber and publisher
        rospy.Subscriber('/battery_status', BatteryStatus, self.battery_status_callback)
        self.error_status_pub = rospy.Publisher('/error_status', ErrorStatus, queue_size=10)

        # Service proxy
        rospy.wait_for_service('/plug_cable')
        self.plug_cable_proxy = rospy.ServiceProxy('/plug_cable', SetBool)

        # Publish error status at 2Hz
        self.error_status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_error_status) 
        

    def battery_status_callback(self, msg):
        percentage = msg.batteryPercentage
        if percentage < self.critical_percent:
            self.call_plug_cable_service(True)  # Start charging
        elif percentage >= self.full_battery:
            self.call_plug_cable_service(False)  # Stop charging


    def call_plug_cable_service(self, state):
        try:
            req = SetBoolRequest()
            req.data = state
            self.plug_cable_proxy(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def publish_error_status(self, event):
        error_msg = ErrorStatus()
        try:
            last_battery_msg = rospy.wait_for_message("/battery_status", BatteryStatus, timeout=1.0)
            error_msg.error = last_battery_msg.batteryPercentage < self.warning_percentage
            error_msg.description = "Robot is about to deplete its battery, don't assign new job" if error_msg.error else ""
        except rospy.ROSException as e:
            rospy.logwarn("No battery status message received yet")
        
        self.error_status_pub.publish(error_msg)


if __name__ == '__main__':
    try:
        AutoChargingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
