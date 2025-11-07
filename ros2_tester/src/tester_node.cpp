//
// Created by fei
//

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <ratio>
#include <iomanip>

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
#include "cobridge_tester/srv/newline_cr.hpp"
#include "cobridge_tester/srv/no_request.hpp"
#include "cobridge_tester/srv/no_response.hpp"
#include "cobridge_tester/srv/null.hpp"

using namespace std::chrono_literals;

class TesterNode : public rclcpp::Node
{
public:
    TesterNode() : Node("cobridge_tester_node")
    {
        rclcpp::QoS qos{rclcpp::KeepLast(10)};
        qos.reliable();
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image", qos);
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/compressed_image", qos);

        remote_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/remote_msgs", 10, 
            std::bind(&TesterNode::remote_callback, this, std::placeholders::_1));

        common_srv_ = this->create_service<cobridge_tester::srv::Common>(
            "/common", 
            std::bind(&TesterNode::common_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        no_request_srv_ = this->create_service<cobridge_tester::srv::NoRequest>(
            "/no_request", 
            std::bind(&TesterNode::no_request_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        no_response_srv_ = this->create_service<cobridge_tester::srv::NoResponse>(
            "/no_response", 
            std::bind(&TesterNode::no_response_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        null_srv_ = this->create_service<cobridge_tester::srv::Null>(
            "/null_srv", 
            std::bind(&TesterNode::null_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        newline_crlf_srv_ = this->create_service<cobridge_tester::srv::NewlineCRLF>(
            "/newline_crlf", 
            std::bind(&TesterNode::newline_CRLF_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        newline_cr_srv_ = this->create_service<cobridge_tester::srv::NewlineCR>(
            "/newline_cr",
            std::bind(&TesterNode::newline_CR_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        running_ = true;
        image_thread_ = std::thread(&TesterNode::imagePublishLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "Tester node initialized with publisher, subscriber and 5 services");
    }

    void setTargetFPS(int fps)
    {
        if (fps > 0 && fps <= 120) {
            target_fps_.store(fps);
            target_interval_.store(std::chrono::microseconds(1000000 / fps));
            RCLCPP_INFO(this->get_logger(), "Target FPS set to %d", fps);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid FPS value: %d, keeping current value", fps);
        }
    }

    ~TesterNode()
    {
        running_ = false;
        if (image_thread_.joinable()) {
            image_thread_.join();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remote_sub_;
    
    rclcpp::Service<cobridge_tester::srv::Common>::SharedPtr common_srv_;
    rclcpp::Service<cobridge_tester::srv::NoRequest>::SharedPtr no_request_srv_;
    rclcpp::Service<cobridge_tester::srv::NoResponse>::SharedPtr no_response_srv_;
    rclcpp::Service<cobridge_tester::srv::Null>::SharedPtr null_srv_;
    rclcpp::Service<cobridge_tester::srv::NewlineCRLF>::SharedPtr newline_crlf_srv_;
    rclcpp::Service<cobridge_tester::srv::NewlineCR>::SharedPtr newline_cr_srv_;

    std::thread image_thread_;
    std::atomic<bool> running_;
    std::atomic<int> target_fps_{30};
    std::atomic<std::chrono::microseconds> target_interval_{std::chrono::microseconds(33333)};

    std::string remote_msgs_;
    std::string node_status_;
    int32_t remote_msgs_draw_count{100};
    int32_t node_status_draw_count{100};

    void imagePublishLoop()
    {
        while (running_) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            publishImage();
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            auto current_interval = target_interval_.load();
            auto sleep_time = current_interval - processing_time;

            if (sleep_time.count() <= 0) {
            RCLCPP_INFO(this->get_logger(),
                "publishImage took %ld us, exceeding target interval of %ld us for %d fps",
                processing_time.count(), current_interval.count(), target_fps_.load());
            }else{
                std::this_thread::sleep_for(sleep_time);
            }
        }
    }

    void publishImage()
    {
        auto now = std::chrono::system_clock::now();
        cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));

        static int frame_count = 0;
        frame_count++;

        cv::putText(image, "ROS2 Test Node", cv::Point(50, 100),
                   cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(image, "Frame: " + std::to_string(frame_count), 
                   cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX,
                   2, cv::Scalar(0, 255, 0), 2);

        std::string timestamp_str = std::to_string(now.time_since_epoch().count());

        auto time_t = std::chrono::system_clock::to_time_t(now);

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << "Time: "<< std::put_time(std::localtime(&time_t), "[%Y-%m-%d %H:%M:%S.");
        ss << std::setfill('0') << std::setw(3) << ms.count() << "]";
        std::string time_str = ss.str();
        
        cv::putText(image, "Timestamp: " + timestamp_str, cv::Point(50, 300),
        cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);

        cv::putText(image, time_str, cv::Point(50, 400),
                   cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);

        if (remote_msgs_draw_count < 20) {
            cv::putText(image, remote_msgs_, cv::Point(50, 550),
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 0, 0),  2);
            remote_msgs_draw_count++;
        }

        if (node_status_draw_count < 20) {
            cv::putText(image, node_status_, cv::Point(50, 980),
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 255),  2);
            node_status_draw_count++;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";

        image_pub_->publish(*msg);

        auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_msg->header.stamp = this->get_clock()->now();
        compressed_msg->header.frame_id = "camera";
        compressed_msg->format = "jpeg";

        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);
        compressed_msg->data.assign(buffer.begin(), buffer.end());
        compressed_pub_->publish(*compressed_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published image frame %d", frame_count);
    }

    void remote_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received remote message: %s", msg->data.c_str());
        remote_msgs_ = msg->data;
        remote_msgs_draw_count = 0;
    }

    void common_callback(
        const std::shared_ptr<cobridge_tester::srv::Common::Request> request,
        std::shared_ptr<cobridge_tester::srv::Common::Response> response)
    {
        node_status_ = "/common service called with data: " + request->data;
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
        response->success = true;
        response->data = "Response to: " + request->data;
    }

    void no_request_callback(
        const std::shared_ptr<cobridge_tester::srv::NoRequest::Request> request,
        std::shared_ptr<cobridge_tester::srv::NoRequest::Response> response)
    {
        node_status_ = "/no_request service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
        response->success = true;
        response->message = "No request service processed successfully";
    }

    void no_response_callback(
        const std::shared_ptr<cobridge_tester::srv::NoResponse::Request> request,
        std::shared_ptr<cobridge_tester::srv::NoResponse::Response> response)
    {
        node_status_ = "/no_response service called with data: " + request->data;
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
    }

    void null_callback(
        const std::shared_ptr<cobridge_tester::srv::Null::Request> request,
        std::shared_ptr<cobridge_tester::srv::Null::Response> response)
    {
        node_status_ = "/null service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());
    }

    void newline_CRLF_callback(
        const std::shared_ptr<cobridge_tester::srv::NewlineCRLF::Request> request,
        std::shared_ptr<cobridge_tester::srv::NewlineCRLF::Response> response)
    {
        node_status_ = "/newline_crlf service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());

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

    void newline_CR_callback(
        const std::shared_ptr<cobridge_tester::srv::NewlineCR::Request> request,
        std::shared_ptr<cobridge_tester::srv::NewlineCR::Response> response)
    {
        node_status_ = "/newline_cr service called";
        node_status_draw_count = 0;
        RCLCPP_INFO(this->get_logger(), "%s", node_status_.c_str());

        response->header.stamp = this->now();
        response->header.frame_id = "cobridge_tester";

        response->name = "CR测试设备";
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
