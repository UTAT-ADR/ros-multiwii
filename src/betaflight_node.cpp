#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <agiros_msgs/Command.h>

#include <msp/Client.hpp>
#include <msp/msp_msg.hpp>

#include <Eigen/Geometry>

double deg2rad(const double deg) {
    return deg/180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad/M_PI * 180.0;
}

class BetaflightNode {
private:
    ros::NodeHandle nh;
    msp::client::Client *client;

    ros::Publisher imu_pub;
    ros::Publisher battery_pub;

    ros::Subscriber command_sub;

public:
    BetaflightNode() {
        nh = ros::NodeHandle("~");
        client = new msp::client::Client();
    }

    ~BetaflightNode() {
        delete client;
    }

    msp::client::Client& bf() const {
        return *client;
    }

    void setup() {
        std::string device;
        std::string command_topic;
        int baudrate = 1000000;
        if(nh.getParam("device", device)) {
            if(!nh.getParam("baudrate", baudrate)) {
                ROS_ERROR("Parameter 'baudrate' not set. Using default baudrate of %i", baudrate);
            }
            ROS_INFO("Connected to FCU at %s", device.c_str());
        }
        else {
            ROS_ERROR("Parameter 'device' not set.");
        }

        if (!nh.getParam("command_topic", command_topic)) {
            ROS_ERROR("Parameter 'command_topic' not set.");
        }
    
        client->start(device, uint(baudrate));

        imu_pub = nh.advertise<sensor_msgs::Imu>("bf/imu", 1);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("bf/battery",1);

        command_sub = nh.subscribe(command_topic, 1, &BetaflightNode::commandCallBack, this); // AERT1234


    }

    void onImu(const msp::msg::RawImu &imu) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "betaflight";

        imu_msg.linear_acceleration.x = imu.acc[0];
        imu_msg.linear_acceleration.y = imu.acc[1];
        imu_msg.linear_acceleration.z = imu.acc[2];

        imu_msg.angular_velocity.x = imu.gyro[0];
        imu_msg.angular_velocity.y = imu.gyro[1];
        imu_msg.angular_velocity.z = imu.gyro[2];

        imu_msg.orientation_covariance[0] = -1.0;

        imu_pub.publish(imu_msg);

    }

    void onBattery(const msp::msg::Analog &analog) {
        sensor_msgs::BatteryState battery;
        battery.header.stamp = ros::Time::now();
        battery.voltage = analog.vbat;
        battery.current = analog.amperage;

        battery_pub.publish(battery);
    }

    void commandCallBack(const agiros_msgs::Command& cmd) {
        msp::msg::SetRawRc rc(msp::FirmwareVariant::BAFL);
        rc.channels.resize(16, 1500);
        rc.channels[0] = 1500; //roll
        rc.channels[1] = 1500; //pitch
        rc.channels[2] = 1500; //yaw
        rc.channels[4] = 1500; //throttle


        client->sendMessageNoWait(rc);

    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Betaflight Bridge");

    BetaflightNode node;

    node.setup();
    ROS_INFO("Betaflight bridge is ready!");

    node.bf().subscribe(&BetaflightNode::onImu, &node, 0.001);
    node.bf().subscribe(&BetaflightNode::onBattery, &node, 0.001);

    ros::spin();

    ros::shutdown();

    return 0;
}