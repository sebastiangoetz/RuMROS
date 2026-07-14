#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseCorrector : public rclcpp::Node {
public:
    PoseCorrector() : Node("pose_corrector") {
        this->declare_parameter<std::string>("namespace", "");
        this->declare_parameter<double>("x_offset", 0.0);
        this->declare_parameter<double>("y_offset", 0.0);

        std::string robotNamespace = this->get_parameter("namespace").as_string();
        x_offset = this->get_parameter("x_offset").as_double();
        y_offset = this->get_parameter("y_offset").as_double();

        std::string input_topic = "/" + robotNamespace + "/internal_pose";
        std::string output_topic = "/" + robotNamespace + "/pose";

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            input_topic, 10,
            std::bind(&PoseCorrector::pose_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Pose Corrector started: namespace %s, x_offset: %.2f, y_offset: %.2f",
                    robotNamespace.c_str(), x_offset, y_offset);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto corrected_msg = *msg;  

        corrected_msg.pose.pose.position.x += x_offset;
        corrected_msg.pose.pose.position.y += y_offset;

        pose_pub_->publish(corrected_msg);

        RCLCPP_INFO(this->get_logger(), "Corrected pose: x=%.2f, y=%.2f",
                    corrected_msg.pose.pose.position.x, corrected_msg.pose.pose.position.y);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    double x_offset;
    double y_offset;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCorrector>());
    rclcpp::shutdown();
    return 0;
}
