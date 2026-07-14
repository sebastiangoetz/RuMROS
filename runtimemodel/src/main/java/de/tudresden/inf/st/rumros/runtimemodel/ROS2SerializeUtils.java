package de.tudresden.inf.st.rumros.runtimemodel;

import java.io.File;

/**
 * Auxiliary class with utility functions to serialize and
 * deserialize various types of ROS messages.
 * 
 * <p>Makes use of the ros2_serialization_utils ROS package.</p>
 */
public class ROS2SerializeUtils {

    // The native methods below are loaded from the serialization library 
    static {
        System.setProperty("LD_LIBRARY_PATH", "/opt/ros/$ROS_DISTRO/lib");
        System.load(new File("build/resources/main/libros2_serialization_utils.so").getAbsolutePath());
    }

    // -------------------------- Native Methods --------------------------

    private static native String deserializeToPoseJson(byte[] byteArray);

    private static native String deserializeToOdometryJson(byte[] byteArray);

    private static native String deserializeToStringJson(byte[] byteArray);

    private static native byte[] serializeTwistJson(String json);

    private static native byte[] serializeStringJson(String json);

    // -------------------------- Public Methods --------------------------

    /**
     * Converts a ROS {@code Odometry} message to a JSON string.
     * @param bytes the message as a byte array
     * @return the deserialized JSON message string
     */
    public static String toOdometryJson(byte[] bytes) {
        return deserializeToOdometryJson(bytes);
    }

    /**
     * Converts a ROS {@code Pose} message to a JSON string.
     * @param bytes the message as a byte array
     * @return the deserialized JSON message string
     */
    public static String toPoseJson(byte[] bytes) {
        return deserializeToPoseJson(bytes);
    }

    /**
     * Converts a ROS {@code String} message to a JSON string.
     * @param bytes the message as a byte array
     * @return the deserialized JSON message string
     */
    public static String toStringJson(byte[] bytes) {
        return deserializeToStringJson(bytes);
    }

    /**
     * Converts a ROS {@code Twist} message to a bytes array,
     * suitable for sending on a topic.
     * @param json the JSON string of the message
     * @return the serialized message byte array
     */
    public static byte[] fromTwistJson(String json) {
        return serializeTwistJson(json);
    }

    /**
     * Converts a ROS {@code String} message to a bytes array,
     * suitable for sending on a topic.
     * @param json the JSON string of the message
     * @return the serialized message byte array
     */
    public static byte[] fromStringJson(String json) {
        return serializeStringJson(json);
    }
}
