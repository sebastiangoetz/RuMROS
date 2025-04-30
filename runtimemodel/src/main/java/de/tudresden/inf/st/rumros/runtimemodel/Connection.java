package de.tudresden.inf.st.rumros.runtimemodel;

public class Connection {

    private final String attribute;
    private final String topic;

    public Connection(String attribute, String topic) {
        this.attribute = attribute;
        this.topic = topic;
    }

    public String getAttribute() {
        return attribute;
    }

    public String getTopic() {
        return topic;
    }
}
