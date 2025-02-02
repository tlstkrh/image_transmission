#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        // YAML로부터 파라미터 로드
        this->declare_parameter<std::string>("rtsp_url", "default_url");
        this->declare_parameter<std::string>("qos_reliability", "best_effort");
        this->declare_parameter<int>("qos_depth", 10);

        this->get_parameter("rtsp_url", rtsp_url_);
        this->get_parameter("qos_reliability", qos_reliability_);
        this->get_parameter("qos_depth", qos_depth_);

        // RTSP 스트림 연결
        cap_.open(rtsp_url_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream: %s", rtsp_url_.c_str());
            return;
        }

        // QoS 설정 (YAML에서 읽은 값에 따라 결정)
        rclcpp::QoS qos_profile(qos_depth_);
        if (qos_reliability_ == "best_effort") {
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        } else {
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", qos_profile);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (cap_.read(frame)) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from RTSP stream.");
        }
    }

    cv::VideoCapture cap_;
    std::string rtsp_url_;
    std::string qos_reliability_;
    int qos_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
