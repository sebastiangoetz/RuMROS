#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseOffsetter : public rclcpp::Node {
public:
    PoseOffsetter() : Node("pose_offsetter") {
        this->declare_parameter<std::string>("namespace", "");
        this->declare_parameter<std::string>("input_topic", "odom_internal");
        this->declare_parameter<std::string>("output_topic", "odom");
        this->declare_parameter<std::string>("message_type", "odometry"); // "odometry" or "pose"
        this->declare_parameter<double>("x_offset", 0.0);
        this->declare_parameter<double>("y_offset", 0.0);

        std::string robotNamespace = this->get_parameter("namespace").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string message_type = this->get_parameter("message_type").as_string();
        x_offset = this->get_parameter("x_offset").as_double();
        y_offset = this->get_parameter("y_offset").as_double();

        if (message_type == "odometry") {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + robotNamespace + "/" + input_topic, 10,
                std::bind(&PoseOffsetter::odom_callback, this, std::placeholders::_1));

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                "/" + robotNamespace + "/" + output_topic, 10);
        } else if (message_type == "pose") {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/" + robotNamespace + "/" + input_topic, 10,
                std::bind(&PoseOffsetter::pose_callback, this, std::placeholders::_1));

            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/" + robotNamespace + "/" + output_topic, 10);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid message type. Use 'odometry' or 'pose'.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Pose offsetter started: namespace %s, input_topic %s, output_topic %s, message_type: %s, x_offset: %.2f, y_offset: %.2f",
                    robotNamespace.c_str(), input_topic.c_str(), output_topic.c_str(), message_type.c_str(), x_offset, y_offset);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto corrected_msg = *msg;  
        corrected_msg.pose.pose.position.x += x_offset;
        corrected_msg.pose.pose.position.y += y_offset;
        odom_pub_->publish(corrected_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto corrected_msg = *msg;  
        corrected_msg.pose.pose.position.x += x_offset;
        corrected_msg.pose.pose.position.y += y_offset;
        pose_pub_->publish(corrected_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    double x_offset;
    double y_offset;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseOffsetter>());
    rclcpp::shutdown();
    return 0;
}
