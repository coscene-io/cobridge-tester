//
// Created by fei
//

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef ROS2_VERSION_JAZZY
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "opencv2/opencv.hpp"

#include "cobridge_tester/srv/common.hpp"
#include "cobridge_tester/srv/newline_crlf.hpp"
#include "cobridge_tester/srv/no_request.hpp"
#include "cobridge_tester/srv/no_response.hpp"
#include "cobridge_tester/srv/null.hpp"

using namespace std::chrono_literals;

class TesterNode : public rclcpp::Node
{
public:
    TesterNode() : Node("cobridge_tester_node")
    {
        // 初始化发布者和订阅者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_image", 10);
        remote_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/remote_msgs", 10, 
            std::bind(&TesterNode::remoteCallback, this, std::placeholders::_1));
        
        // 初始化服务
        common_srv_ = this->create_service<cobridge_tester::srv::Common>(
            "/common", 
            std::bind(&TesterNode::commonCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        no_request_srv_ = this->create_service<cobridge_tester::srv::NoRequest>(
            "/no_request", 
            std::bind(&TesterNode::noRequestCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        no_response_srv_ = this->create_service<cobridge_tester::srv::NoResponse>(
            "/no_response", 
            std::bind(&TesterNode::noResponseCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        null_srv_ = this->create_service<cobridge_tester::srv::Null>(
            "/null_srv", 
            std::bind(&TesterNode::nullCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        newline_crlf_srv_ = this->create_service<cobridge_tester::srv::NewlineCRLF>(
            "/newline_crlf", 
            std::bind(&TesterNode::newlineCrlfCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建定时器，定期发布图像
        timer_ = this->create_wall_timer(100ms, std::bind(&TesterNode::publishImage, this));
        
        RCLCPP_INFO(this->get_logger(), "Tester node initialized with publisher, subscriber and 5 services");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remote_sub_;
    
    rclcpp::Service<cobridge_tester::srv::Common>::SharedPtr common_srv_;
    rclcpp::Service<cobridge_tester::srv::NoRequest>::SharedPtr no_request_srv_;
    rclcpp::Service<cobridge_tester::srv::NoResponse>::SharedPtr no_response_srv_;
    rclcpp::Service<cobridge_tester::srv::Null>::SharedPtr null_srv_;
    rclcpp::Service<cobridge_tester::srv::NewlineCRLF>::SharedPtr newline_crlf_srv_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::string remote_msgs_;
    std::string node_status_;
    int32_t remote_msgs_draw_count{100};
    int32_t node_status_draw_count{100};

    // 发布图像的回调函数
    void publishImage()
    {
        // 创建一个简单的彩色图像
        cv::Mat image(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // 在图像上绘制一些内容
        static int frame_count = 0;
        frame_count++;
        
        // 添加一些动态内容
        cv::putText(image, "ROS2 Test Image", cv::Point(50, 150),
                   cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(image, "Frame: " + std::to_string(frame_count), 
                   cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX,
                   2, cv::Scalar(0, 255, 0), 2);
        
        // 添加当前时间戳
        auto now = this->now();
        std::string time_str = std::to_string(now.seconds());
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
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = now;
        msg->header.frame_id = "camera";
        
        // 发布图像
        image_pub_->publish(*msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published image frame %d", frame_count);
    }
    
    // 接收远程消息的回调函数
    void remoteCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received remote message: %s", msg->data.c_str());
        remote_msgs_ = msg->data;
        remote_msgs_draw_count = 0;
    }
    
    // Common服务的回调函数
    void commonCallback(
        const std::shared_ptr<cobridge_tester::srv::Common::Request> request,
        std::shared_ptr<cobridge_tester::srv::Common::Response> response)
    {
        node_status_ = "/common service called with data: " + request->data;
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
        response->success = true;
        response->data = "Response to: " + request->data;
    }
    
    // No_Request服务的回调函数
    void noRequestCallback(
        const std::shared_ptr<cobridge_tester::srv::NoRequest::Request> request,
        std::shared_ptr<cobridge_tester::srv::NoRequest::Response> response)
    {
        node_status_ = "/no_request service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
        response->success = true;
        response->message = "No request service processed successfully";
    }
    
    // No_Response服务的回调函数
    void noResponseCallback(
        const std::shared_ptr<cobridge_tester::srv::NoResponse::Request> request,
        std::shared_ptr<cobridge_tester::srv::NoResponse::Response> response)
    {
        node_status_ = "/no_response service called with data: " + request->data;
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
    }
    
    // Null服务的回调函数
    void nullCallback(
        const std::shared_ptr<cobridge_tester::srv::Null::Request> request,
        std::shared_ptr<cobridge_tester::srv::Null::Response> response)
    {
        node_status_ = "/null service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
    }
    
    // Newline_CRLF服务的回调函数
    void newlineCrlfCallback(
        const std::shared_ptr<cobridge_tester::srv::NewlineCRLF::Request> request,
        std::shared_ptr<cobridge_tester::srv::NewlineCRLF::Response> response)
    {
        node_status_ = "/newline_crlf service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
        
        // 填充响应数据
        response->header.stamp = this->now();
        response->header.frame_id = "cobridge_tester";
        
        response->name = "CRLF测试设备";
        response->serial_number = "SN12345678";
        response->firmware_version = "v1.2.3";
        response->supported_min_sdk_version = "v1.0.0";
        response->hardware_version = "HW2.0";
        response->success = true;
        response->message = "CRLF服务调用成功";
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
