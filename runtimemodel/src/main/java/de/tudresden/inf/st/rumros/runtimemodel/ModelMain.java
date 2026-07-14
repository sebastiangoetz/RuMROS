package de.tudresden.inf.st.rumros.runtimemodel;

import com.fasterxml.jackson.core.*;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.JsonNodeType;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.apache.commons.jexl3.*;

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
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

import static java.util.concurrent.TimeUnit.SECONDS;

/**
 * This is the main entry point for the runtime model, representing the controller
 * component in the package diagram.
 * 
 * <p>This component initializes RAGConnect from
 * configuration files, creates a concrete AST {@link Model} and links its attributes
 * accordingly. During runtime, is responsible for periodically executing the main loop
 * of the model and communicating with the web UI, i.e. transmitting the updated model
 * as well as executing actions selected on the UI.</p>
 */
public class ModelMain {
    
    /** The socket address of the web UI's UDP socket */
    private static SocketAddress addr = null;

    /** The size of the received message buffer */
    private static final int bufferSize = 4096;

    /** The host address of of the web UI */
    private static final String WEBAPP_HOST = "127.0.0.1";  // Standard loopback interface address (localhost)

    /** The port of the web UI */
    private static final int WEBAPP_PORT = 65431;  // Port to listen on (non-privileged ports are > 1023)

    /** The concrete runtime model instance */
    private static Model model = new Model();

    /**
     * A map of {@link Action}s to threads processing their result status,
     * intended for processing {@link AwaitableResult}s
     */
    private static final Map<Thread, Action> awaitableThreads = new ConcurrentHashMap<>();

    /**
     * The application entry point, which initializes MQTT and UDP communication, as well as the concrete model.
     * @param args The arguments to be passed
     * @throws Exception if any errors are encountered
     */
    public static void main(String[] args) throws Exception {
        //Run with either Gazebo or Argos depending on what was specified
        //This is required to load the correct mqtt-connections.json file
        String simulator = (args.length > 0) ? args[0] : "gazebo";
        String configFileName = "/mqtt-connections." + simulator + ".json"; //Keep slash at start as its being loaded as a resource

        // AST initialization
        model = model.init();

        for (Action action : model.getActions()) {
            action.setResult(new EmptyResult());
        }

        setupRagConnect(configFileName);

        DatagramSocket udpServerSocket;
        try {
            udpServerSocket = new DatagramSocket(WEBAPP_PORT, InetAddress.getByName(WEBAPP_HOST));

            // Receive from Webapp
            Thread receiveThread = new Thread(() -> receiveMessages(udpServerSocket));
            receiveThread.start();

            // Publish to Webapp
            Thread publishThread = new Thread(() -> publishMessages(udpServerSocket));
            publishThread.start();

            //Execute JastAdd model startup routine
            model.start();
            if (Model.TIME_STEP > 0)
                Thread.sleep(Model.TIME_STEP);

            while (true) {
                // Execute model step function and sleep for specified number of seconds
                model.step();
                if (Model.TIME_STEP > 0)
                    Thread.sleep(Model.TIME_STEP);
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Generic helper to remove any {@link ASTNode} object from a {@link JastAddList}.
     * @param <T> the concrete {@link ASTNode} type
     * @param list the list from which the object should be removed
     * @param element the object to remove from the list
     */
    private static <T extends ASTNode<?>> void removeObjectFromList(JastAddList<?> list, T element) {
        for (int i = 0; i < list.getNumChild(); i++) {
            if (list.getChild(i) == element) {
                list.removeChild(i);
                return;
            }
        }
    }

    /**
     * Awaits the result of an {@link Action} and updates an {@link AwaitableResult} object as soon as it is completed.
     * @param action the action to await
     * @param res the result object to synchronize
     */
    private static void awaitResult(Action action, AwaitableResult res) {
        try {
            Result r = null;
            while (!Thread.currentThread().isInterrupted()) {
                synchronized (model) {
                    if (res.isAcknowledged(model)) {
                        // Received ACK, execute logic
                        r = res.execute(model);
                        removeObjectFromList(model.getPendingResults(), res); //Remove pending result

                        // Remove ACK message from received messages
                        int removeIndex = -1;
                        JastAddList<CommandMsg> receivedMsgs = model.getCommandManager().getReceivedMsgs();
                        for (int i = 0; i < receivedMsgs.getNumChild(); i++) {
                            CommandMsg msg = receivedMsgs.getChild(i);
                            if (msg != null && msg.gettype() == 0) {
                                if (((AckMsg)msg).getuuid().equals(res.getuuid())) {
                                    removeIndex = i;
                                    break;
                                }
                            }
                        }   
                        if (removeIndex >= 0)
                            receivedMsgs.removeChild(removeIndex);
                        
                        break;
                    }
                    Thread.sleep(1000);
                }
            }
            synchronized (action) {
                if (r == null)
                    action.setResult(new Result(ActionResultType.TECHNICAL_ERROR, "No result received", 2000));
                else
                    action.setResult(r);
            }

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.err.println(e.getMessage());
            synchronized (action) {
                action.setResult(new Result(ActionResultType.TECHNICAL_ERROR, "Awaitable action interrupted", 2000));
            }
            return;
        }
    }

    /**
     * Creates a thread that awaits an {@link Action}, by executing {@link ModelMain#awaitResult} in the thread.
     * @param action the action to be awaited
     * @param res the result to be synchronized with the action
     */
    public static void createAwaitThread(Action action, AwaitableResult res) {
        // Start thread that waits for result
        Thread t = new Thread(() -> {
            awaitResult(action, res);
        });
        awaitableThreads.put(t, action);
        
        t.start();
        action.setResult(res); // Waiting..
    }

    /**
     * Processing function for incoming UDP messages from the web UI.
     * @param udpServerSocket the socket through which messages are received
     */
    private static void receiveMessages(DatagramSocket udpServerSocket) {
        byte[] buffer = new byte[bufferSize];

        // Continuous loop - run in a separate thread
        while (true) {
            try {
                // Extract packet from socket if available
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                udpServerSocket.receive(packet); // BLOCKS
                addr = packet.getSocketAddress();
                String msg = new String(packet.getData(), 0, packet.getLength());

                // Handle various custom message types as defined by the web UI
                // explicitly. Finally, handle Action executions in the else clause.
                if (msg.equals("hello")) {
                    // Send hello
                   sendModel(udpServerSocket);
                } else if (msg.equals("toggleBehaviorModel")) {
                    // Toggle behavior model running state
                    BehaviorModel bm = model.getBehaviorModel();
                    if (bm.getrunning())
                        bm.stop();
                    else
                        bm.run();
                } else {
                    // Handle actions selected on the UI
                    // Receive Goal
                    ObjectMapper objectMapper = new ObjectMapper();
                    String json = new String(packet.getData(), 0, packet.getLength());
                    JsonNode jsonNode = objectMapper.readTree(json);

                    // Read values
                    String actionId = jsonNode.get("id").asText();
                    Map<String, String> stringInputs = objectMapper.convertValue(jsonNode.get("inputs"), new TypeReference<>() {
                    });

                    Action action = StreamSupport.stream(model.getActionList().spliterator(), false).filter(a -> a.getid().equals(actionId)).findFirst().get();

                    Object[] inputs = new Object[action.getNumInput() + 1];
                    Class<?>[] inputClasses = new Class<?>[action.getNumInput() + 1];

                    // Parse action parameters
                    // Always add model
                    inputs[0] = model;
                    inputClasses[0] = Model.class;

                    for (int i = 0; i < action.getNumInput(); i++) {
                        Input input = action.getInput(i);
                        if (!stringInputs.containsKey(input.getid())) {
                            throw new IllegalArgumentException("Missing input '" + input.getid() + "' for action '" + actionId + "'!");
                        }

                        // Primitives
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
                        } else if (input.gettype().equals(Boolean.class.getName())) {
                            inputs[i+1] = stringInputs.get(input.getid()).isEmpty() ? null : Boolean.parseBoolean(stringInputs.get(input.getid()));
                            stringInputs.remove(input.getid());
                            inputClasses[i+1] = Boolean.class;
                            continue;
                        }

                        // Is instance of ASTNode
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
                            // Use reflection to find method associated with the Action and invoke it
                            Method method = Action.class.getDeclaredMethod(actionId, inputClasses);
                            method.setAccessible(true);

                            Object returnValue = method.invoke(action, inputs);

                            if (!(returnValue instanceof Result)) {
                                throw new IllegalArgumentException("Invalid action '" + actionId + "', it should return an object of class Result!");
                            }
                            
                            Result res = (Result) returnValue;
                            //Handling AwaitableResults has been moved to the step method. While this does
                            //make the UI less responsive (no feedback), it avoids the dependency on JastAdd,
                            //keeping ModelMain more generic. To restore the UI functionality, uncomment
                            //the lines below. However, the handling in step() is still required in order to
                            //properly process messages when an Action is started via the behavior model, doubling
                            //the handling and making it possibly thread-unsafe, so this is not advised.
                            // if (res instanceof AwaitableResult) {
                            //     createAwaitThread(action, (AwaitableResult)res);
                            //     break;
                            // }

                            // Set result of simple action (awaitables yield ActionResultType.WAIT instead)
                            action.setResult(res);
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

    /**
     * Processing function for outgoing UDP messages to the web UI.
     * @param udpServerSocket the socket through which messages are sent
     */
    private static void publishMessages(DatagramSocket udpServerSocket) {
        Action delayedAction = null;
        while (true) {
            if (addr != null) {
                // Handle awaitable actions first
                if (delayedAction != null) {
                    // Awaitable actions have their state reset here
                    // Once this is done, they can be considered finished
                    // and removed from any queues in the model
                    delayedAction.setResult(new EmptyResult());
                    delayedAction = null;
                }

                // Update number of awaiting threads
                // Required to synchronously send action state
                List<Thread> finished = new ArrayList<>();
                for (Map.Entry<Thread, Action> entry : awaitableThreads.entrySet()) {
                    Thread thread = entry.getKey();
                    if (!thread.isAlive()) {
                        finished.add(thread);
                    }
                }

                for (Thread thread : finished) {
                    Action action = awaitableThreads.remove(thread);
                    if (action != null) {
                        delayedAction = action;
                    }
                }
                
                sendModel(udpServerSocket);
            }
            try {
                TimeUnit.SECONDS.sleep(1); // Should be adjusted depending on machine performance
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    /**
     * Transmits the {@link Model} as a serialized JSON message to the web UI via the UDP socket.
     * @param udpServerSocket the socket through which to send the model
     */
    private static void sendModel(DatagramSocket udpServerSocket) {
        try {
            String json = getModelJson();
            udpServerSocket.send(new DatagramPacket(json.getBytes(), json.getBytes().length, addr));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Wrapper for cleanly serializing the {@link Model} object to a JSON string.
     * @return the JSON string representing the model
     * @throws SerializationException if serialization fails unexpectedly
     * @throws IOException on I/O-related issues
     */
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

    /**
     * Expands the types of a node in the JSON representation of the model by finding parent node names,
     * such that instead of having types like {@code Robot}, after expanding it contains {@code Robot|SwarmUnit}.
     * @param node the JSON node to expand
     */
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
                    // Is ASTNode subclass
                    ((ObjectNode) node).put("type", className.toString());
                }

                if (node.has("children")) {
                    for (JsonNode child : node.get("children")) {
                        if (child.isContainerNode()) {
                            expandNodeTypes(child);
                        }
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

    /**
     * Sets up RAGConnect according to the given configuration.
     * @param configFileName the name of the RAGConnect configuration file
     * @throws IOException if errors occurred reading the configuration file
     * @throws NoSuchMethodException if a reflectively invoked method does not exist
     * @throws InvocationTargetException if a specified connection class does not exist
     * @throws IllegalAccessException if a specified connection class or method can't be accessed
     */
    private static void setupRagConnect(String configFileName) throws IOException, NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        // RAGConnect initialization
        model.ragconnectSetupMqttWaitUntilReady(2, SECONDS);

        InputStream mqttConnectionsInputStream = ModelMain.class.getResourceAsStream(configFileName);

        // Check if mqtt-connections.*.json exists in the resource dir, if not -> skip
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

                    // Connect outputs
                    for (Connection output : outputs) {
                        String topic = replaceAttributes(output.getTopic(), node);
                        Method m = node.getClass().getMethod("connect" + output.getAttribute(), String.class, boolean.class);
                        m.invoke(node, topic, true);
                    }

                    // Connect inputs
                    for (Connection input : inputs) {
                        String topic = replaceAttributes(input.getTopic(), node);
                        Method m = node.getClass().getMethod("connect" + input.getAttribute(), String.class);
                        m.invoke(node, topic);
                    }
                }
            }
        }
    }

    /**
     * Given a class name, finds concrete {@link ASTNode} instances of this class in the given model subtree.
     * @param node the root node of the subtree
     * @param clazzName the name of the class to find
     * @return a list of all found nodes, empty if none were found
     */
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

    /**
     * Checks whether a class is an instance of a class with the given name, including superclass names.
     * @param clazz the class to check
     * @param name the name to check against
     * @return true if the class has the given name or a superclass of the given name, false otherwise
     */
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

    /**
     * Recursively finds a node in the given AST subtree by type and unique ID.
     * @param current the AST subtree to search in
     * @param targetId the ID of the node to find
     * @param clazz the type of the node to find
     * @return the first found node object, or null if no matching objects were found
     */
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

    /**
     * Replaces attribute placeholders in the given text with values resolved from the specified AST node.
     * 
     * <p>Placeholders must follow the pattern {@code ${attribute}} or {@code ${attribute.subAttribute}},
     * where each segment corresponds to a JavaBean-style getter method. For example, {@code ${name}}
     * resolves via {@code getName()}, and {@code ${parent.id}} resolves via {@code getParent().getId()}.</p>
     * @param text the text containing attribute placeholders
     * @param node the root node used to resolve attribute values
     * @return the text with all attribute placeholders replaced by their resolved values
     * @throws NoSuchMethodException if a required getter method does not exist
     * @throws InvocationTargetException if a getter method invocation fails
     * @throws IllegalAccessException if a getter method is not accessible
     */
    private static String replaceAttributes(String text, ASTNode<?> node) throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        Pattern expr = Pattern.compile("\\$\\{([^}]+)\\}");
        Matcher matcher = expr.matcher(text);
        while (matcher.find()) {
            String[] attributeNames = matcher.group(1).split("\\.");

            // Evaluate attribute
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

    /**
     * Evaluates the specified condition using the Apache Commons JEXL expression engine.
     * This is used to resolve templates of 'condition' node values in mqtt-connections.*.json.
     * @param condition the JEXL expression to evaluate
     * @return a boolean representing the evaluation result
     */

    private static boolean evaluateCondition(String condition) {
        JexlEngine jexl = new JexlBuilder().create();
        JexlExpression expression = jexl.createExpression(condition);
        JexlContext context = new MapContext();

        Object result = expression.evaluate(context);
        return result instanceof Boolean && (Boolean) result;
    }
}
