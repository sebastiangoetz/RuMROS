package de.tudresden.inf.st.rumros.runtimemodel;

import com.fasterxml.jackson.core.*;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.JsonNodeType;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.apache.commons.jexl3.*;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketAddress;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import static java.util.concurrent.TimeUnit.SECONDS;

public class ModelMain {
    private static SocketAddress addr = null;
    private static final int bufferSize = 4096;
    private static final String WEBAPP_HOST = "127.0.0.1";  // Standard loopback interface address (localhost)
    private static final int WEBAPP_PORT = 65431;  // Port to listen on (non-privileged ports are > 1023)

    private static Model model = new Model();

    public static void main(String[] args) throws Exception {
        // AST initialization
        model = model.init();

        for (Action action : model.getActions()) {
            action.setResult(new EmptyResult());
        }

        setupRagConnect();

        DatagramSocket udpServerSocket;
        try {
            udpServerSocket = new DatagramSocket(WEBAPP_PORT, InetAddress.getByName(WEBAPP_HOST));

            // Receive from Webapp
            Thread receiveThread = new Thread(() -> receiveMessages(udpServerSocket));
            receiveThread.start();

            // Publish to Webapp
            Thread publishThread = new Thread(() -> publishMessages(udpServerSocket));
            publishThread.start();

            while (true) {
                Thread.sleep(1000);
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    private static void receiveMessages(DatagramSocket udpServerSocket) {
        byte[] buffer = new byte[bufferSize];
        while (true) {
            try {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                udpServerSocket.receive(packet); // BLOCKS
                addr = packet.getSocketAddress();
                String msg = new String(packet.getData(), 0, packet.getLength());
                if (msg.equals("hello")) {
                    // send hello
                   sendModel(udpServerSocket);
                } else {
                    // receive Goal
                    ObjectMapper objectMapper = new ObjectMapper();
                    String json = new String(packet.getData(), 0, packet.getLength());
                    JsonNode jsonNode = objectMapper.readTree(json);

                    //read values
                    String actionId = jsonNode.get("id").asText();
                    Map<String, String> stringInputs = objectMapper.convertValue(jsonNode.get("inputs"), new TypeReference<>() {
                    });

                    Action action = StreamSupport.stream(model.getActionList().spliterator(), false).filter(a -> a.getid().equals(actionId)).findFirst().get();

                    Object[] inputs = new Object[action.getNumInput() + 1];
                    Class<?>[] inputClasses = new Class<?>[action.getNumInput() + 1];

                    //always add model
                    inputs[0] = model;
                    inputClasses[0] = Model.class;

                    for (int i = 0; i < action.getNumInput(); i++) {
                        Input input = action.getInput(i);
                        if (!stringInputs.containsKey(input.getid())) {
                            throw new IllegalArgumentException("Missing input '" + input.getid() + "' for action '" + actionId + "'!");
                        }

                        //primitives
                        if (input.gettype().equals(String.class.getName())) {
                            inputs[i+1] = stringInputs.get(input.getid());
                            stringInputs.remove(input.getid());
                            inputClasses[i+1] = String.class;
                            continue;
                        } else if (input.gettype().equals(Integer.class.getName())) {
                            inputs[i+1] = stringInputs.get(input.getid()).isEmpty() ? null : Integer.parseInt(stringInputs.get(input.getid()));
                            stringInputs.remove(input.getid());
                            inputClasses[i+1] = Integer.class;
                            continue;
                        } else if (input.gettype().equals(Double.class.getName())) {
                            inputs[i+1] = stringInputs.get(input.getid()).isEmpty() ? null : Double.parseDouble(stringInputs.get(input.getid()));
                            stringInputs.remove(input.getid());
                            inputClasses[i+1] = Double.class;
                            continue;
                        }

                        //is instance of astnode
                        String targetId = stringInputs.get(input.getid());
                        Class<?> clazz = Class.forName(input.gettype());
                        Object object = findNode(model, targetId, clazz);
                        inputs[i+1] = object;
                        inputClasses[i+1] = Class.forName(input.gettype());
                    }

                    int threadCollisionRetries = 3; //workaround
                    while (threadCollisionRetries > 0) {
                        threadCollisionRetries--;
                        try {
                            Method method = Action.class.getDeclaredMethod(actionId, inputClasses);
                            method.setAccessible(true);

                            Object returnValue = method.invoke(action, inputs);

                            if (!(returnValue instanceof Result)) {
                                throw new IllegalArgumentException("Invalid action '" + actionId + "', it should return an object of class Result!");
                            }

                            action.setResult((Result) returnValue);
                            sendModel(udpServerSocket);
                            action.setResult(new EmptyResult());
                            break;
                        } catch (NoSuchMethodException e) {
                            String parameters = Arrays.stream(inputClasses).map(c -> c.getSimpleName() + ".class").collect(Collectors.joining(", "));
                            String errorText = "The method for " + actionId + "(" + parameters + ") is not correctly defined in class Action.";
                            System.err.println(errorText);
                            Result errorResult = new Result(ActionResultType.TECHNICAL_ERROR, errorText, 10000);
                            action.setResult(errorResult);
                            sendModel(udpServerSocket);
                            action.setResult(new EmptyResult());
                            break;
                        } catch (Error error) {
                            if (error.getMessage().equals("Top of handler stack does not match at pop!")) {
                                System.err.println("Collision with other thread, retrying...");
                            } else {
                                String errorText = "An error occurred when executing the action: " + error.getMessage();
                                Result errorResult = new Result(ActionResultType.TECHNICAL_ERROR, errorText, 10000);
                                action.setResult(errorResult);
                                sendModel(udpServerSocket);
                                action.setResult(new EmptyResult());
                                throw error;
                            }
                        } catch (Exception e) {
                            String errorText = "An error occurred when executing the action: " + e.getMessage();
                            Result errorResult = new Result(ActionResultType.TECHNICAL_ERROR, errorText, 10000);
                            action.setResult(errorResult);
                            sendModel(udpServerSocket);
                            action.setResult(new EmptyResult());
                            throw e;
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private static void publishMessages(DatagramSocket udpServerSocket) {
        while (true) {
            if (addr != null) {
                sendModel(udpServerSocket);
            }
            try {
                TimeUnit.SECONDS.sleep(1); // depends on the performance of your PC
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private static void sendModel(DatagramSocket udpServerSocket) {
        try {
            String json = getModelJson();
            udpServerSocket.send(new DatagramPacket(json.getBytes(), json.getBytes().length, addr));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static String getModelJson() throws SerializationException, IOException {
        ObjectMapper objectMapper = new ObjectMapper();

        JsonFactory factory = new JsonFactory();
        StringWriter stringWriter = new StringWriter();
        JsonGenerator generator = factory.createGenerator(stringWriter);
        model.serialize(generator, null);
        generator.close();
        ObjectNode modelJsonNode = (ObjectNode) objectMapper.readTree(stringWriter.toString());

        for(JsonNode child : modelJsonNode.get("children")) {
            if(child.isContainerNode()) {
                expandNodeTypes(child);
            }
        }

        String json = modelJsonNode.get("children").toString();

        return json;
    }

    private static void expandNodeTypes(JsonNode node) {
        if(node.isObject()) {
            try {
                StringBuilder className = new StringBuilder(node.get("type").asText());
                Class<?> clazz = Class.forName(ASTNode.class.getPackageName() + "." + className);

                while (clazz.getSuperclass() != null && !clazz.getSuperclass().equals(ASTNode.class)) {
                    clazz = clazz.getSuperclass();
                    className.append("|");
                    className.append(clazz.getSimpleName());
                }

                if (clazz.getSuperclass() != null) {
                    //is ASTNode subclass
                    ((ObjectNode) node).put("type", className.toString());
                }

                for (JsonNode child : node.get("children")) {
                    if (child.isContainerNode()) {
                        expandNodeTypes(child);
                    }
                }

            } catch (ClassNotFoundException e) {
                throw new RuntimeException(e);
            }
        } else if (node.isArray()) {
            for (JsonNode child : node) {
                expandNodeTypes(child);
            }
        } else {
            throw new UnsupportedOperationException("Unsupported node type: " + node.getClass().getName());
        }
    }

    private static void setupRagConnect() throws IOException, NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        // RagConnect initialization
        model.ragconnectSetupMqttWaitUntilReady(2, SECONDS);

        InputStream mqttConnectionsInputStream = ModelMain.class.getResourceAsStream("/mqtt-connections.json");

        //Check if mqtt-connections.json exists in the resource dir, if not -> skip
        if(mqttConnectionsInputStream != null) {
            ObjectMapper objectMapper = new ObjectMapper();
            JsonNode jsonNode = objectMapper.readTree(mqttConnectionsInputStream);

            for (JsonNode connection : jsonNode) {
                assert connection.getNodeType() == JsonNodeType.OBJECT;
                assert connection.has("class");

                String clazz = connection.get("class").asText();

                List<Connection> inputs = new ArrayList<>();
                if (connection.has("inputs")) {
                    for (JsonNode input : connection.get("inputs")) {
                        inputs.add(new Connection(
                                input.get("attribute").asText(),
                                input.get("topic").asText()));
                    }
                }

                List<Connection> outputs = new ArrayList<>();
                if (connection.has("outputs")) {
                    for (JsonNode output : connection.get("outputs")) {
                        outputs.add(new Connection(
                                output.get("attribute").asText(),
                                output.get("topic").asText()));
                    }
                }

                List<ASTNode<?>> nodesToConnect = findNodesOfClass(model, clazz);

                for (ASTNode<?> node : nodesToConnect) {

                    if (connection.has("condition")) {
                        String condition = connection.get("condition").asText();
                        condition = replaceAttributes(condition, node);

                        // Skip this connection if the condition is false
                        if (!evaluateCondition(condition)) {
                            continue;
                        }
                    }

                    //connect outputs
                    for (Connection output : outputs) {
                        String topic = replaceAttributes(output.getTopic(), node);
                        Method m = node.getClass().getMethod("connect" + output.getAttribute(), String.class, boolean.class);
                        m.invoke(node, topic, true);
                    }

                    //connect inputs
                    for (Connection input : inputs) {
                        String topic = replaceAttributes(input.getTopic(), node);
                        Method m = node.getClass().getMethod("connect" + input.getAttribute(), String.class);
                        m.invoke(node, topic);
                    }
                }
            }
        }
    }

    private static List<ASTNode<?>> findNodesOfClass(ASTNode<?> node, String clazzName) {
        List<ASTNode<?>> result = new ArrayList<>();

        for (int i = 0; i < node.getNumChild(); i++) {
            ASTNode<?> child = node.getChild(i);
            if (child instanceof JastAddList) {
                JastAddList<?> list = (JastAddList<?>) child;
                if (list.getNumChild() == 0) {
                    continue;
                }
                for (ASTNode<?> childElement : list) {
                    if (isSubclassOrClassWithName(childElement.getClass(), clazzName)) {
                        //found
                        result.add(childElement);
                    }
                    result.addAll(findNodesOfClass(childElement, clazzName));
                }
            } else {
                if (isSubclassOrClassWithName(child.getClass(), clazzName)) {
                    //found
                    result.add(child);
                    break;
                }
                result.addAll(findNodesOfClass(child, clazzName));
            }
        }
        return result;
    }

    public static boolean isSubclassOrClassWithName(Class<?> clazz, String name) {
        Class<?> current = clazz;
        while (current != null) {
            if (current.getSimpleName().equalsIgnoreCase(name)) {
                return true;
            }
            current = current.getSuperclass();
        }
        return false;
    }

    public static Object findNode(ASTNode<?> current, String targetId, Class<?> clazz) {
        for (int i = 0; i < current.getNumChild(); i++) {
            ASTNode<?> child = current.getChild(i);
            if (child == null) {
                continue;
            }
            if (clazz.isInstance(child) && child.unique$Id.equals(targetId)) {
                return child;
            }

            if (child.getNumChild() > 0) {
                Object result = findNode(child, targetId, clazz);
                if (result != null) {
                    return result;
                }
            }
        }

        return null;
    }

    private static String replaceAttributes(String text, ASTNode<?> node) throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        Pattern expr = Pattern.compile("\\$\\{([^}]+)\\}");
        Matcher matcher = expr.matcher(text);
        while (matcher.find()) {
            String[] attributeNames = matcher.group(1).split("\\.");

            //evaluate attribute
            Object currentValue = node;
            for(String attributeName : attributeNames) {
                    Method getter = currentValue.getClass().getMethod("get" + attributeName);
                    currentValue = getter.invoke(currentValue);
            }

            Pattern subexpr = Pattern.compile(Pattern.quote(matcher.group(0)));

            String replacement = (currentValue instanceof Number) ? currentValue.toString() : "\"" + currentValue + "\"";

            text = subexpr.matcher(text).replaceAll(replacement);
            matcher = expr.matcher(text);
        }
        return text;
    }

    private static boolean evaluateCondition(String condition) {
        JexlEngine jexl = new JexlBuilder().create();
        JexlExpression expression = jexl.createExpression(condition);
        JexlContext context = new MapContext();

        Object result = expression.evaluate(context);
        return result instanceof Boolean && (Boolean) result;
    }
}
