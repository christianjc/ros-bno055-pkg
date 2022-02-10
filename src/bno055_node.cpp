/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * 
*/

#include <csignal>
#include <memory>

#include <ros/ros.h>

#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <sensor_msgs/Imu.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>

#include <bno055_driver/bno055_driver.h>


class BNO055Node {
    private:
        ros::NodeHandle* node_hadle;
        ros::NodeHandle* node_hadle_private;
        std::unique_ptr<bno055_imu::BNO055Driver> bno_imu;

        std::string param_device;
        int param_address;
        double param_rate;
        std::string param_frame_id;

        diagnostic_msgs::DiagnosticStatus current_status;

        ros::Publisher pub_data;

        std::unique_ptr<ros::Rate> rate;

        int seq;
    public:
        BNO055Node(int argc, char* argv[]);
        void run();
        bool publishData();
        void stop();
};


BNO055Node::BNO055Node(int argc, char* argv[]) {
    ros::init(argc, argv, "bno055_node");
    node_hadle = new ros::NodeHandle();
    node_hadle_private = new ros::NodeHandle("~");

    if(!node_hadle || !node_hadle_private) {
        ROS_FATAL("Failed to initialize node handles");
        ros::shutdown();
        return;
    }

    node_hadle_private->param("device", param_device, (std::string)"/dev/i2c-1");
    node_hadle_private->param("address", param_address, (int)BNO055_ADDRESS);
    node_hadle_private->param("frame_id", param_frame_id, (std::string)"bno_imu");
    node_hadle_private->param("rate", param_rate, (double)100);
 
    bno_imu = std::make_unique<bno055_imu::BNO055Driver>(param_device, param_address);

    bno_imu->init();

    pub_data = node_hadle->advertise<sensor_msgs::Imu>("data", 1);

    seq = 0;

    rate =std::make_unique<ros::Rate>(param_rate);
}


void BNO055Node::run() {
    int num_of_fails = 0;
    while(ros::ok()) {
        rate->sleep();
        std::cout << "Called the published node" << std::endl;
        if(!publishData()) {
            std::cout << "could not publish" << std::endl;
            num_of_fails++;
            if (num_of_fails > 10) {
                BNO055Node::stop();
            }
        }
    }
}

bool BNO055Node::publishData() {
    bno055_imu::imu_data_t data;

    try {
        data = bno_imu->read_imu_data();
    } catch(const std::runtime_error& e) {
        ROS_WARN_STREAM(e.what());
    }

    ros::Time time = ros::Time::now();

    sensor_msgs::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;
    msg_data.header.seq = seq;

    /*!
     * Assign to Quaternion
     * See
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     * 3.6.5.5 Orientation (Quaternion)
     */
    const double scale = (1.0 / (1 << 14));

    msg_data.orientation.w = (double)data.quaternion_w * scale;
    msg_data.orientation.x = (double)data.quaternion_x * scale;
    msg_data.orientation.y = (double)data.quaternion_y * scale;
    msg_data.orientation.z = (double)data.quaternion_z * scale;
    msg_data.linear_acceleration.x = (double)data.linear_acceleration_x / 100.0;
    msg_data.linear_acceleration.y = (double)data.linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.z = (double)data.linear_acceleration_z / 100.0;
    msg_data.angular_velocity.x = (double)data.angular_velocity_x / 900.0;
    msg_data.angular_velocity.y = (double)data.angular_velocity_y / 900.0;
    msg_data.angular_velocity.z = (double)data.angular_velocity_z / 900.0;

    pub_data.publish(msg_data);

    return true;
}

void BNO055Node::stop() {
    ROS_INFO("stopping");
    if(pub_data) pub_data.shutdown();
}

int main(int argc, char *argv[]) {
    BNO055Node node(argc, argv);
    node.run();
    return 0;
}
