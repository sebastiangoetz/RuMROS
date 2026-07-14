package de.tudresden.inf.st.rumros.mqttmapper;

import org.apache.commons.jexl3.JexlBuilder;
import org.apache.commons.jexl3.JexlContext;
import org.apache.commons.jexl3.JexlEngine;
import org.apache.commons.jexl3.JexlExpression;
import org.apache.commons.jexl3.MapContext;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.SafeConstructor;
import org.yaml.snakeyaml.representer.Representer;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import de.tudresden.inf.st.rumros.runtimemodel.ASTNode;
import de.tudresden.inf.st.rumros.runtimemodel.JastAddList;
import de.tudresden.inf.st.rumros.runtimemodel.Model;

import org.yaml.snakeyaml.nodes.Tag;

import java.io.*;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This class offers functionality to generate a YAML configuration file for the MQTT client ROS node
 * from a given RAGConnect configuration file and information on robot counts from the JastAdd model
 * specification.
 */
public class MqttTopicMapper {
    /**
     * Parses a RAGConnect config file to a list of mappings.
     * @param jsonPath the path to the config file
     * @return a list of classes with MQTT mappings
     */
    public static List<Map<String, Object>> parseJsonConfig(String filePath) {
        ObjectMapper mapper = new ObjectMapper();
        try {
            return mapper.readValue(new File(filePath), new TypeReference<List<Map<String, Object>>>(){});
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Updates the mqtt_client's config.yaml according to the RAGConnect configuration file.
     * This method takes a model instance in order to evaluate the conditions specified in
     * the configuration file, and creates topics for the found number of robots accordingly.
     * @param yamlPath the path to mqtt_client's configuration file
     * @param jsonPath the path to RAGConnects' configuration file
     * @param model an initialized instance of the AST root node
     * @throws IOException if the RAGConnect configuration file cannot be read or mqtt_client's
     * configuration file cannot be read and written
     */
    public static void updateConfigYaml(String yamlPath, String jsonPath, Model model) throws IOException {

        // Load original YAML config
        InputStream yamlInputStream = new FileInputStream(yamlPath);
        LoaderOptions lOptions = new LoaderOptions();
        Yaml yaml = new Yaml(new SafeConstructor(lOptions));
        Map<String, Object> yamlConfig = yaml.load(yamlInputStream);
        yamlInputStream.close();

        // Build YAML output object
        Map<String, Object> root = (Map<String, Object>) yamlConfig.get("/**/*");
        if (root == null) {
            root = new LinkedHashMap<>();
            yamlConfig.put("/**/*", root);
        }

        Map<String, Object> rosParams = (Map<String, Object>) root.get("ros__parameters");
        if (rosParams == null) {
            rosParams = new LinkedHashMap<>();
            root.put("ros__parameters", rosParams);
        }

        Map<String, Object> bridge = (Map<String, Object>) rosParams.get("bridge");
        if (bridge == null) {
            bridge = new LinkedHashMap<>();
            rosParams.put("bridge", bridge);
        }

        Map<String, Object> ros2mqtt = (Map<String, Object>) bridge.get("ros2mqtt");
        if (ros2mqtt == null) {
            ros2mqtt = new LinkedHashMap<>();
            bridge.put("ros2mqtt", ros2mqtt);
        } else {
            ros2mqtt.clear();
        }
        Set<String> ros2mqttTopics = new LinkedHashSet<>();

        Map<String, Object> mqtt2ros = (Map<String, Object>) bridge.get("mqtt2ros");
        if (mqtt2ros == null) {
            mqtt2ros = new LinkedHashMap<>();
            bridge.put("mqtt2ros", mqtt2ros);
        } else {
            mqtt2ros.clear();
        }
        Set<String> mqtt2rosTopics = new LinkedHashSet<>();



        // Parse RAGConnect JSON file
        List<Map<String, Object>> jsonConfig = parseJsonConfig(jsonPath);

        if (jsonConfig != null) {
            try {
                for (Map<String, Object> classEntry : jsonConfig) {

                    String className = (String) classEntry.get("class");

                    // Resolve model nodes
                    List<ASTNode<?>> nodes = MappingUtils.findNodesOfClass(model, className);
                    if (nodes == null || nodes.isEmpty()) {
                        continue;
                    }

                    List<Map<String, Object>> inputs = (List<Map<String, Object>>) classEntry.get("inputs");
                    List<Map<String, Object>> outputs = (List<Map<String, Object>>) classEntry.get("outputs");
                    String conditionTemplate = (String) classEntry.get("condition");

                    // Match nodes with classes in JSON config
                    for (ASTNode<?> node : nodes) {

                        // Evaluate JEXL condition
                        if (conditionTemplate != null) {
                            String cond = MappingUtils.replaceAttributes(conditionTemplate, node);
                            if (!MappingUtils.evaluateCondition(cond)) {
                                continue;
                            }
                        }

                        // Process ROS2MQTT (inputs)
                        if (inputs != null) {
                            for (Map<String, Object> input : inputs) {

                                String topicTemplate = (String) input.get("topic");

                                // Resolve template
                                String expanded = MappingUtils.replaceAttributes(topicTemplate, node);
                                String cleanTopic = MappingUtils.stripMqttPrefix(expanded);

                                // Create YAML entry
                                String rosKey = "/" + cleanTopic;
                                ros2mqtt.putIfAbsent(
                                        rosKey,
                                        Map.of("mqtt_topic", cleanTopic)
                                );

                                ros2mqttTopics.add(rosKey);
                            }
                        }

                        // Process MQTT2ROS (outputs)
                        if (outputs != null) {
                            for (Map<String, Object> output : outputs) {

                                String topicTemplate = (String) output.get("topic");
                                String type = (String) output.get("type");

                                // Resolve template
                                String expanded = MappingUtils.replaceAttributes(topicTemplate, node);
                                String cleanTopic = MappingUtils.stripMqttPrefix(expanded);

                                // Create YAML entry
                                mqtt2ros.putIfAbsent(
                                        cleanTopic,
                                        Map.of(
                                                "ros_topic", "/" + cleanTopic,
                                                "ros_type", type
                                        )
                                );

                                mqtt2rosTopics.add(cleanTopic);
                            }
                        }
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
                System.err.println("Error while generating MQTT config from model + JSON");
                System.exit(1);
            }
        } else {
            throw new IOException("Failed to load RAGConnect config file (" + jsonPath + ")");
        }


        // Add to YAML
        if (!ros2mqttTopics.isEmpty())
            ros2mqtt.put("ros_topics", new ArrayList<>(ros2mqttTopics));

        if (!mqtt2rosTopics.isEmpty())
            mqtt2ros.put("mqtt_topics", new ArrayList<>(mqtt2rosTopics));

        // If node is empty, remove it, as mqtt_client will error otherwise
        if (ros2mqtt.isEmpty())
            bridge.remove("ros2mqtt");
        if (mqtt2ros.isEmpty())
            bridge.remove("mqtt2ros");
        if (bridge.isEmpty())
            rosParams.remove("bridge");

        // Dump updated YAML back to file
        DumperOptions dOptions = new DumperOptions();
        dOptions.setIndent(2);
        dOptions.setPrettyFlow(true);
        dOptions.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK);

        Representer representer = new Representer(dOptions);
        representer.addClassTag(Map.class, Tag.MAP);

        Yaml outYaml = new Yaml(representer, dOptions);
        FileWriter writer = new FileWriter(yamlPath);
        outYaml.dump(yamlConfig, writer);
        writer.close();
    }

    /**
     * This is the entry point for the configuration mapper, called by a Gradle build task
     * as a preprocessing step.
     * @param args the arguments to pass to the configuration mapper: names of mqtt_client's
     * and RAGConnect's configuration files respectively
     * @throws IOException if the configuration files cannot be accessed or written
     * @throws RuntimeException upon encountering miscellaneous errors 
     */
    public static void main(String[] args) throws IOException, RuntimeException {
        // Read RAGConnect config file and update mqtt_client config
        Model model = new Model().init();
        updateConfigYaml(args[0], args[1], model);
    }
}
