#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        //url set
        rtsp_url_ = "rtsp://192.168.144.25:8554/main.264";
        //std::string pipeline = "rtspsrc location=" + rtsp_url_ + " latency=0 ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false";
        //std::string pipeline = "rtspsrc location=" + rtsp_url_ + " latency=0 protocols=tcp ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false";
        std::string pipeline = "rtspsrc location=" + rtsp_url_ + " latency=100 protocols=udp ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink max-buffers=1 drop=true sync=false";

        
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream with GStreamer pipeline: %s", pipeline.c_str());
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to RTSP stream using GStreamer pipeline");
        }
        
        // QoS set
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        // topic name
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", qos_profile);
        
        // FPS set
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (cap_.read(frame)) {
            // test gui
            cv::imshow("RTSP Stream", frame);
            cv::waitKey(1);
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from RTSP stream.");
        }
    }

    cv::VideoCapture cap_;
    std::string rtsp_url_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}