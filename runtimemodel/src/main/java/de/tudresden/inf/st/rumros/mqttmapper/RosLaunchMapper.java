package de.tudresden.inf.st.rumros.mqttmapper;

import java.util.Map;
import java.io.File;
import java.util.List;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.tudresden.inf.st.rumros.runtimemodel.Model;

/**
 * This class offers functionality to generate a configuration file for ROS launch, required
 * in order for the ROS layer to initialize robot nodes of the right type and group mappings.
 */
public class RosLaunchMapper {
    /**
     * Generates a configuration file from a map in a given static field of the {@link Model} class.
     * @param configFile the config file to write
     * @param fieldName the name of the field to read the configuration map from
     * @throws Exception if any errors are encountered
     */
    public static void generateConfig(File configFile, String fieldName) throws Exception {
        // Get field from JastAdd generated class using reflection
        Map<Integer, List<Integer>> config = (Map<Integer, List<Integer>>) MappingUtils.getField("Model", fieldName);
        
        ObjectMapper mapper = new ObjectMapper();
        mapper.writerWithDefaultPrettyPrinter().writeValue(configFile, config);
    }

    /**
     * Generates a JSON configuration file from the given {@link Object}.
     * @param configFile the config file to write
     * @param configMap the object to serialize
     * @throws Exception if any errors are encountered
     */
    public static void generateConfig(File configFile, Object configMap) throws Exception {
        ObjectMapper mapper = new ObjectMapper();
        mapper.writerWithDefaultPrettyPrinter().writeValue(configFile, configMap);
    }

    /**
     * This is the entry point for the configuration mapper, called by a Gradle build task
     * as a preprocessing step.
     * 
     * <p>When called by the Gradle task, it generates two configuration files in the
     * {@code config} folder inside the ROS workspace:</p>
     * <ul>
     * <li><em>swarm_config.json</em> for swarm group - robot ID mappings</li>
     * <li><em>type_config.json</em> for robot type - robot ID mappings</li>
     * </ul>
     * @param args the paths and names of the configuration files to generate
     * @throws Exception on any encountered error
     */
    public static void main(String[] args) throws Exception {
        File swarmConfigFile = new File(args[0]);
        File typeConfigFile = new File(args[1]);
        File parentDir = swarmConfigFile.getParentFile();

        // Create config directory in ROS workspace if it doesn't exist
        if (parentDir != null && !parentDir.exists()) {
            if (!parentDir.mkdirs()) {
                throw new RuntimeException("Failed to create directory: " + parentDir.getAbsolutePath());
            }
        }

        // Load and initialize model to derive swarm and type configs
        Model model = new Model().init();
        Map<Integer, List<Integer>> swarmConfig = model.getSwarmConfig();
        Map<String, List<Integer>> typeConfig = model.getTypeConfig();
        
        // Write config files
        generateConfig(swarmConfigFile, swarmConfig);
        generateConfig(typeConfigFile, typeConfig);
    }
}
