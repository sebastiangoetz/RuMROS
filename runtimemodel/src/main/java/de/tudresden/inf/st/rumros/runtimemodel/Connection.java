package de.tudresden.inf.st.rumros.runtimemodel;

/**
 * Represents a connection between the runtime model and the MQTT client node,
 * as parsed from the MQTT client configuration files (mqtt-connections.*.json).
 * 
 * <p>Each connection maps an AST attribute to a MQTT topic.</p>
 */
public class Connection {

    /** The name of the mapped AST node's attribute */
    private final String attribute;

    /** The MQTT topic name associated to the attribute */
    private final String topic;

    /**
     * Initializes the class parameters.
     * @param attribute name of the AST node attribute to be mapped
     * @param topic MQTT topic on which attribute values are sent or received
     */
    public Connection(String attribute, String topic) {
        this.attribute = attribute;
        this.topic = topic;
    }

    /**
     * Returns the mapped AST node attribute name.
     * @return the name of the mapped attribute
     */
    public String getAttribute() {
        return attribute;
    }

    /**
     * Returns the MQTT topic name.
     * @return the topic name
     */
    public String getTopic() {
        return topic;
    }
}
