#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MapSaver : public rclcpp::Node
{
public:
    MapSaver() : Node("map_saver")
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("turtlebot3_gazebo");
        map_save_path_ = package_share_directory + "/map";  
        
        // Create a client for the serialize_map service
        serialize_client_ = this->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");

        RCLCPP_INFO(this->get_logger(), "Started map_saver");

        // Set a timer to call the save_map_callback every 10 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&MapSaver::save_map_callback, this)
        );
    }

private:
    void save_map_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timer triggered for save_map_callback");
        if (!serialize_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service /slam_toolbox/serialize_map not available");
            return;
        }

        // Prepare the service request
        auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
        request->filename = map_save_path_;

        // Call the service asynchronously
        auto future = serialize_client_->async_send_request(request);
        auto future_status = future.wait_for(std::chrono::seconds(8));

        if (future_status == std::future_status::ready) {
            auto response = future.get();
            if (response->result == 0) {
                RCLCPP_INFO(this->get_logger(), "Map serialized successfully at %s", map_save_path_.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to serialize map");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
        }

    }

    std::string map_save_path_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
