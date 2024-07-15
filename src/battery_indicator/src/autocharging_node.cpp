#include "ros/ros.h"
#include "battery_indicator/BatteryStatus.h"
#include "battery_indicator/ErrorStatus.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/SetBoolRequest.h"
#include "std_srvs/SetBoolResponse.h"

class AutoChargingNode 
{
public:
    AutoChargingNode() 
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~"); 

        // ROS Params
        pnh.param<float>("critical_percent", critical_percent, 15.0);
        pnh.param<float>("full_battery", full_battery, 100.0);
        pnh.param<float>("warning_percentage", warning_percentage, 25.0);

        // Battery Status Subscriber
        batteryStatusSubscriber = nh.subscribe("/battery_status", 10, &AutoChargingNode::batteryStatusCallback, this);
    
        // Error Status Publisher
        errorStatusPublisher = nh.advertise<battery_indicator::ErrorStatus>("/error_status", 100);

        // Service Proxy
        ros::service::waitForService("/plug_cable", ros::Duration(2.0));

        plugCableProxy = nh.serviceClient<std_srvs::SetBool>("/plug_cable");

        // Publish error status at 2 Hz
        errorStatusTimer = nh.createTimer(ros::Duration(0.5), &AutoChargingNode::publishErrorStatusCallback, this);

        // ROS_INFO("Constructor Completed for Autocharging node c++!");
    }

    void publishErrorStatusCallback (const ros::TimerEvent& event) 
    {
        battery_indicator::ErrorStatus errorStatusMsg;
        try
        {
            battery_indicator::BatteryStatus::ConstPtr batteryStatusPtr;
            batteryStatusPtr = ros::topic::waitForMessage<battery_indicator::BatteryStatus>("/battery_status", ros::Duration(2.0));

            errorStatusMsg.error = batteryStatusPtr->batteryPercentage < this->warning_percentage;
            if (errorStatusMsg.error) 
            {
                errorStatusMsg.description = "Robot is about to deplete its battery, don't assign new job";
            } 
            else
            {
                errorStatusMsg.description = "All Well!";
            }
            
        }
        catch(const std::exception& e)
        {
            ROS_WARN("Exception getting message from topic /battery_status");
        }

        errorStatusPublisher.publish(errorStatusMsg);
    } 

    void batteryStatusCallback (const battery_indicator::BatteryStatus::ConstPtr& msg)
    {
        float currentBatteryPercentage = msg->batteryPercentage;
        if (currentBatteryPercentage < critical_percent) 
        {
            // Start Charging
            callPlugCableService(true);
        } 
        else if (currentBatteryPercentage >= full_battery) 
        {
            // Stop Charging
            callPlugCableService(false);
        }
    }

    void callPlugCableService (bool state) 
    {
        try
        {
            std_srvs::SetBoolRequest req;
            std_srvs::SetBoolResponse res;
            req.data = state;

            plugCableProxy.call(req, res);
        }
        catch(const std::exception& e)
        {
            ROS_WARN("Exception calling Plub Cable Service");
        }
    }
private:
    float critical_percent;
    float full_battery;
    float warning_percentage;

    ros::Subscriber batteryStatusSubscriber;
    ros::Publisher errorStatusPublisher;
    ros::ServiceClient plugCableProxy;
    ros::Timer errorStatusTimer;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "autocharging_node");

    AutoChargingNode ac;

    ros::spin();
    return 0;
}