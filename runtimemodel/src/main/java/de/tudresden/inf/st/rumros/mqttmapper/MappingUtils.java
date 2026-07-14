package de.tudresden.inf.st.rumros.mqttmapper;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.jexl3.JexlBuilder;
import org.apache.commons.jexl3.JexlContext;
import org.apache.commons.jexl3.JexlEngine;
import org.apache.commons.jexl3.JexlExpression;
import org.apache.commons.jexl3.MapContext;

import de.tudresden.inf.st.rumros.runtimemodel.ASTNode;
import de.tudresden.inf.st.rumros.runtimemodel.JastAddList;

/** Utility class which holds methods to process MQTT configuration attributes. */
public class MappingUtils {

    /**
     * Removes the mqtt:// prefix from MQTT urls.
     * @param topic the MQTT topic string to remove the prefix from
     * @return the extracted topic string
     */
    public static String stripMqttPrefix(String topic) {
        if (topic.startsWith("mqtt://")) {
            int idx = topic.indexOf('/', "mqtt://".length());
            if (idx >= 0) {
                return topic.substring(idx + 1);
            }
        }
        return topic;
    }

    /**
     * Given a class name, finds concrete {@link ASTNode} instances of this class in the given model subtree.
     * This method is a variant of the traversal method in ModelMain, which some adjustments to fully traverse
     * the tree.
     * @param node the root node of the subtree
     * @param clazzName the name of the class to find
     * @return a list of all found nodes (may contain duplicates), empty if none were found
     */
    public static List<ASTNode<?>> findNodesOfClass(ASTNode<?> root, String clazzName) {
        List<ASTNode<?>> result = new ArrayList<>();
        if (root == null) {
            return result;
        }

        // Check the root itself
        if (isSubclassOrClassWithName(root.getClass(), clazzName)) {
            result.add(root);
        }

        // Traverse children safely
        int numChildren = root.getNumChild();
        for (int i = 0; i < numChildren; i++) {
            ASTNode<?> child = root.getChild(i);

            // Adjustment: avoid exception on null child (leaf node)
            if (child == null) {
                continue;
            }

            if (child instanceof JastAddList<?>) {
                JastAddList<?> list = (JastAddList<?>) child;

                for (ASTNode<?> element : list) {
                    if (element == null) {
                        continue;
                    }

                    // Check element itself
                    if (isSubclassOrClassWithName(element.getClass(), clazzName)) {
                        result.add(element);
                    }

                    // Recurse
                    result.addAll(findNodesOfClass(element, clazzName));
                }
            } else {
                // Check child itself
                if (isSubclassOrClassWithName(child.getClass(), clazzName)) {
                    result.add(child);
                }

                // Recurse
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
    public static String replaceAttributes(String text, ASTNode<?> node) throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
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
    public static boolean evaluateCondition(String condition) {
        JexlEngine jexl = new JexlBuilder().create();
        JexlExpression expression = jexl.createExpression(condition);
        JexlContext context = new MapContext();

        Object result = expression.evaluate(context);
        return result instanceof Boolean && (Boolean) result;
    }

    /**
     * Helper method to fetch a value from a static field in a class of the runtime
     * model via reflection.
     * @param className the name of the class
     * @param fieldName the name of the field
     * @return the retrieved object
     */
    public static Object getField(String className, String fieldName) {
        try {
            //Load the class
            String fullClassName = "de.tudresden.inf.st.rumros.runtimemodel." + className;
            Class<?> clazz = Class.forName(fullClassName);
            Field field = clazz.getField(fieldName);

            //Static field, null object
            return field.get(null);
        } catch (ClassNotFoundException e) {
            System.err.println("Class not found: " + className);
        } catch (NoSuchFieldException e) {
            System.err.println("Field " + fieldName + " not found in class: " + className);
        } catch (IllegalAccessException e) {
            System.err.println("Cannot access field " + fieldName + " in class: " + className);
        }
        return -1;
    }
}
