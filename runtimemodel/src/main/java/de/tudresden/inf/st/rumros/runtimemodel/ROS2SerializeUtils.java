package de.tudresden.inf.st.rumros.runtimemodel;

import java.io.File;

public class ROS2SerializeUtils {

    static {
        System.setProperty("LD_LIBRARY_PATH", "/opt/ros/iron/lib");
        System.load(new File("build/resources/main/libros2_serialization_utils.so").getAbsolutePath());
    }

    // -------------------------- Native Methods --------------------------

    private static native String deserializeToPoseJson(byte[] byteArray);

    private static native String deserializeToOdometryJson(byte[] byteArray);

    private static native byte[] serializeTwistJson(String json);

    // -------------------------- Public Methods --------------------------

    public static String toOdometryJson(byte[] bytes) {
        return deserializeToOdometryJson(bytes);
    }

    public static String toPoseJson(byte[] bytes) {
        return deserializeToPoseJson(bytes);
    }

    public static byte[] fromTwistJson(String json) {
        return serializeTwistJson(json);
    }
}
