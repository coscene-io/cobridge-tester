//
// Created by fei on 25-6-10.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cobridge_tester/Common.h>
#include <cobridge_tester/NoRequest.h>
#include <cobridge_tester/NoResponse.h>
#include <cobridge_tester/Null.h>
#include <cobridge_tester/NewlineCRLF.h>
#include <cobridge_tester/NewlineCR.h>

class TesterNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    ros::Publisher compressed_pub_;
    ros::Subscriber remote_sub_;
    
    ros::ServiceServer common_srv_;
    ros::ServiceServer no_request_srv_;
    ros::ServiceServer no_response_srv_;
    ros::ServiceServer null_srv_;
    ros::ServiceServer newline_crlf_srv_;
    
    ros::Timer timer_;

    std::string remote_msgs_;
    std::string node_status_;
    int32_t remote_msgs_draw_count{100};
    int32_t node_status_draw_count{100};

    void publish_image(const ros::TimerEvent&) {
        cv::Mat image(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));

        static int frame_count = 0;
        frame_count++;

        cv::putText(image, "ROS Test Image", cv::Point(50, 150),
                   cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(image, "Frame: " + std::to_string(frame_count), 
                   cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX,
                   2, cv::Scalar(0, 255, 0), 2);

        auto now = std::chrono::system_clock::now();
        std::string time_str = std::to_string(now.time_since_epoch().count());
        auto time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), " [%Y-%m-%d %H:%M:%S]");
        time_str += ss.str();

        cv::putText(image, "Time: " + time_str, cv::Point(50, 350),
                   cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);

        if (remote_msgs_draw_count < 20) {
            cv::putText(image, remote_msgs_, cv::Point(50, 450),
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 0, 0),  2);
            remote_msgs_draw_count++;
        }

        if (node_status_draw_count < 20) {
            cv::putText(image, node_status_, cv::Point(50, 980),
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 255),  2);
            node_status_draw_count++;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera";
        image_pub_.publish(msg);


        auto compressed_msg = std::make_shared<sensor_msgs::CompressedImage>();
        compressed_msg->header.stamp = ros::Time::now();
        compressed_msg->header.frame_id = "camera";
        compressed_msg->format = "jpeg";

        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);
        compressed_msg->data.assign(buffer.begin(), buffer.end());
        compressed_pub_->publish(*compressed_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published image frame %d", frame_count);
        
        ROS_DEBUG("Published image frame %d", frame_count);
    }

    void remote_callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received remote message: %s", msg->data.c_str());
        remote_msgs_ = msg->data;
        remote_msgs_draw_count = 0;
    }

    bool common_callback(cobridge_tester::Common::Request &req,
                        cobridge_tester::Common::Response &res) {
        node_status_ = "/common service called with data: " + req.data;
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        res.success = true;
        res.data = "Response to: " + req.data;
        return true;
    }

    bool no_request_callback(cobridge_tester::NoRequest::Request &req,
                          cobridge_tester::NoRequest::Response &res) {
        node_status_ = "/no_request service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        res.success = true;
        res.message = "No request service processed successfully";
        return true;
    }

    bool no_response_callback(cobridge_tester::NoResponse::Request &req,
                           cobridge_tester::NoResponse::Response &res) {
        node_status_ = "/no_response service called with data: " + req.data;
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        return true;
    }

    bool null_callback(cobridge_tester::Null::Request &req,
                     cobridge_tester::Null::Response &res) {
        node_status_ = "/null_srv service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        return true;
    }

    bool newline_CRLF_callback(cobridge_tester::NewlineCRLF::Request &req,
                          cobridge_tester::NewlineCRLF::Response &res) {
        node_status_ = "/newline_crlf service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());

        res.header.stamp = ros::Time::now();
        res.header.frame_id = "cobridge_tester";
        
        res.name = "CRLF测试设备";
        res.serial_number = "SN12345678";
        res.firmware_version = "v1.2.3";
        res.supported_min_sdk_version = "v1.0.0";
        res.hardware_version = "HW2.0";
        res.success = true;
        res.message = "CRLF服务调用成功";
        
        return true;
    }
    
public:
    TesterNode() {
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/raw_image", 10);
        compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/camera/compressed_image", 10);
        remote_sub_ = nh_.subscribe("/remote_msgs", 10, &TesterNode::remote_callback, this);

        common_srv_ = nh_.advertiseService("/common", &TesterNode::common_callback, this);
        no_request_srv_ = nh_.advertiseService("/no_request", &TesterNode::no_request_callback, this);
        no_response_srv_ = nh_.advertiseService("/no_response", &TesterNode::no_response_callback, this);
        null_srv_ = nh_.advertiseService("/null_srv", &TesterNode::null_callback, this);
        newline_crlf_srv_ = nh_.advertiseService("/newline_crlf", &TesterNode::newline_CRLF_callback, this);

        timer_ = nh_.createTimer(ros::Duration(0.1), &TesterNode::publish_image, this);
        
        ROS_INFO("Tester node initialized with publisher, subscriber and 5 services");
    }
    
    void run() {
        ROS_INFO("Tester node running...");
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cobridge_tester_node");
    
    TesterNode tester;
    tester.run();
    
    return 0;
}
