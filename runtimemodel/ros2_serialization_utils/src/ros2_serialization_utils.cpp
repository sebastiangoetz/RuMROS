#include <jni.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <vector>
#include <iostream>
#include "nlohmann/json.hpp" // Include the JSON library

// Function to convert Pose message to JSON
nlohmann::json poseToJson(const geometry_msgs::msg::PoseStamped &pose_msg) {
    nlohmann::json j;
    j["position"]["x"] = pose_msg.pose.position.x;
    j["position"]["y"] = pose_msg.pose.position.y;
    j["position"]["z"] = pose_msg.pose.position.z;
    j["orientation"]["x"] = pose_msg.pose.orientation.x;
    j["orientation"]["y"] = pose_msg.pose.orientation.y;
    j["orientation"]["z"] = pose_msg.pose.orientation.z;
    j["orientation"]["w"] = pose_msg.pose.orientation.w;
    return j;
}

extern "C"
JNIEXPORT jstring JNICALL Java_de_tudresden_inf_st_rumros_runtimemodel_ROS2SerializeUtils_deserializeToPoseJson
  (JNIEnv *env, jclass clazz, jbyteArray byteArray) {
    jsize length = env->GetArrayLength(byteArray);
    jbyte* bytes = env->GetByteArrayElements(byteArray, 0);

    std::vector<uint8_t> byte_vector(bytes, bytes + length);
    
    try {
        // Create a SerializedMessage
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(byte_vector.size());
        memcpy(serialized_msg.get_rcl_serialized_message().buffer, byte_vector.data(), byte_vector.size());
        serialized_msg.get_rcl_serialized_message().buffer_length = byte_vector.size();

        // Deserialize to Pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
        serializer.deserialize_message(&serialized_msg, &pose_msg);

        // Convert Pose message to JSON
        nlohmann::json j = poseToJson(pose_msg);

        // Convert JSON to string and return as jstring
        std::string json_str = j.dump();
        return env->NewStringUTF(json_str.c_str());
    } catch (const std::exception &e) {
        std::cerr << "Failed to deserialize message: " << e.what() << "\n";
    }

    env->ReleaseByteArrayElements(byteArray, bytes, 0);
    return env->NewStringUTF("");
}

// Function to convert Odometry message to JSON
nlohmann::json odometryToJson(const nav_msgs::msg::Odometry &odom_msg) {
    nlohmann::json j;
    j["position"]["x"] = odom_msg.pose.pose.position.x;
    j["position"]["y"] = odom_msg.pose.pose.position.y;
    j["position"]["z"] = odom_msg.pose.pose.position.z;
    j["orientation"]["x"] = odom_msg.pose.pose.orientation.x;
    j["orientation"]["y"] = odom_msg.pose.pose.orientation.y;
    j["orientation"]["z"] = odom_msg.pose.pose.orientation.z;
    j["orientation"]["w"] = odom_msg.pose.pose.orientation.w;
    j["twist"]["linear"]["x"] = odom_msg.twist.twist.linear.x;
    j["twist"]["linear"]["y"] = odom_msg.twist.twist.linear.y;
    j["twist"]["linear"]["z"] = odom_msg.twist.twist.linear.z;
    j["twist"]["angular"]["x"] = odom_msg.twist.twist.angular.x;
    j["twist"]["angular"]["y"] = odom_msg.twist.twist.angular.y;
    j["twist"]["angular"]["z"] = odom_msg.twist.twist.angular.z;
    return j;
}

extern "C"
JNIEXPORT jstring JNICALL Java_de_tudresden_inf_st_rumros_runtimemodel_ROS2SerializeUtils_deserializeToOdometryJson
  (JNIEnv *env, jclass clazz, jbyteArray byteArray) {
    jsize length = env->GetArrayLength(byteArray);
    jbyte* bytes = env->GetByteArrayElements(byteArray, 0);

    std::vector<uint8_t> byte_vector(bytes, bytes + length);

    try {
        // Create a SerializedMessage
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(byte_vector.size());
        memcpy(serialized_msg.get_rcl_serialized_message().buffer, byte_vector.data(), byte_vector.size());
        serialized_msg.get_rcl_serialized_message().buffer_length = byte_vector.size();

        // Deserialize to Odometry message
        nav_msgs::msg::Odometry odom_msg;
        rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
        serializer.deserialize_message(&serialized_msg, &odom_msg);

        // Convert Odometry message to JSON
        nlohmann::json j = odometryToJson(odom_msg);

        // Convert JSON to string and return as jstring
        std::string json_str = j.dump();
        return env->NewStringUTF(json_str.c_str());
    } catch (const std::exception &e) {
        std::cerr << "Failed to deserialize message: " << e.what() << "\n";
    }

    env->ReleaseByteArrayElements(byteArray, bytes, 0);
    return env->NewStringUTF("");
}

// Function to convert JSON to Twist message
geometry_msgs::msg::Twist jsonToTwist(const nlohmann::json &j) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = j["linear"]["x"];
    twist_msg.linear.y = j["linear"]["y"];
    twist_msg.linear.z = j["linear"]["z"];
    twist_msg.angular.x = j["angular"]["x"];
    twist_msg.angular.y = j["angular"]["y"];
    twist_msg.angular.z = j["angular"]["z"];
    return twist_msg;
}

extern "C"
JNIEXPORT jbyteArray JNICALL Java_de_tudresden_inf_st_rumros_runtimemodel_ROS2SerializeUtils_serializeTwistJson
  (JNIEnv *env, jclass clazz, jstring json) {
    const char *nativeString = env->GetStringUTFChars(json, JNI_FALSE);
    std::string jsonString(nativeString);
    env->ReleaseStringUTFChars(json, nativeString);

    try {
        // Parse JSON string to Twist message
        nlohmann::json j = nlohmann::json::parse(jsonString);
        geometry_msgs::msg::Twist twist_msg = jsonToTwist(j);

        // Serialize Twist message
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
        serializer.serialize_message(&twist_msg, &serialized_msg);

        // Convert serialized message to byte array
        jbyteArray byteArray = env->NewByteArray(serialized_msg.get_rcl_serialized_message().buffer_length);
        env->SetByteArrayRegion(byteArray, 0, serialized_msg.get_rcl_serialized_message().buffer_length, reinterpret_cast<jbyte*>(serialized_msg.get_rcl_serialized_message().buffer));

        return byteArray;
    } catch (const std::exception &e) {
        std::cerr << "Failed to serialize message: " << e.what() << "\n";
    }
    
    return nullptr;
}

