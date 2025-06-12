//
// Created by fei on 25-6-10.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cobridge_tester/Common.h>
#include <cobridge_tester/NoRequest.h>
#include <cobridge_tester/NoResponse.h>
#include <cobridge_tester/Null.h>
#include <cobridge_tester/NewlineCRLF.h>

class TesterNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
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

    // 发布图像的回调函数
    void publishImage(const ros::TimerEvent&) {
        // 创建一个简单的彩色图像
        cv::Mat image(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // 在图像上绘制一些内容
        static int frame_count = 0;
        frame_count++;
        
        // 添加一些动态内容
        cv::putText(image, "ROS Test Image", cv::Point(50, 150),
                   cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(image, "Frame: " + std::to_string(frame_count), 
                   cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX,
                   2, cv::Scalar(0, 255, 0), 2);
        
        // 添加当前时间戳
        std::string time_str = std::to_string(ros::Time::now().toSec());
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
        
        // 转换为ROS图像消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera";
        
        // 发布图像
        image_pub_.publish(msg);
        
        ROS_DEBUG("Published image frame %d", frame_count);
    }
    
    // 接收远程消息的回调函数
    void remoteCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Received remote message: %s", msg->data.c_str());
        remote_msgs_ = msg->data;
        remote_msgs_draw_count = 0;
    }
    
    // Common服务的回调函数
    bool commonCallback(cobridge_tester::Common::Request &req,
                        cobridge_tester::Common::Response &res) {
        node_status_ = "/common service called with data: " + req.data;
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        res.success = true;
        res.data = "Response to: " + req.data;
        return true;
    }
    
    // No_Request服务的回调函数
    bool noRequestCallback(cobridge_tester::NoRequest::Request &req,
                          cobridge_tester::NoRequest::Response &res) {
        node_status_ = "/no_request service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        res.success = true;
        res.message = "No request service processed successfully";
        return true;
    }
    
    // No_Response服务的回调函数
    bool noResponseCallback(cobridge_tester::NoResponse::Request &req,
                           cobridge_tester::NoResponse::Response &res) {
        node_status_ = "/no_response service called with data: " + req.data;
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        return true;
    }
    
    // Null服务的回调函数
    bool nullCallback(cobridge_tester::Null::Request &req,
                     cobridge_tester::Null::Response &res) {
        node_status_ = "/null_srv service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        return true;
    }
    
    // Newline_CRLF服务的回调函数
    bool newlineCrlfCallback(cobridge_tester::NewlineCRLF::Request &req,
                          cobridge_tester::NewlineCRLF::Response &res) {
        node_status_ = "/newline_crlf service called";
        node_status_draw_count = 0;
        ROS_INFO("%s", node_status_.c_str());
        
        // 填充响应数据
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
        // 初始化发布者和订阅者
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera_image", 10);
        remote_sub_ = nh_.subscribe("/remote_msgs", 10, &TesterNode::remoteCallback, this);
        
        // 初始化服务
        common_srv_ = nh_.advertiseService("/common", &TesterNode::commonCallback, this);
        no_request_srv_ = nh_.advertiseService("/no_request", &TesterNode::noRequestCallback, this);
        no_response_srv_ = nh_.advertiseService("/no_response", &TesterNode::noResponseCallback, this);
        null_srv_ = nh_.advertiseService("/null_srv", &TesterNode::nullCallback, this);
        newline_crlf_srv_ = nh_.advertiseService("/newline_crlf", &TesterNode::newlineCrlfCallback, this);
        
        // 创建定时器，定期发布图像
        timer_ = nh_.createTimer(ros::Duration(0.1), &TesterNode::publishImage, this);
        
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
