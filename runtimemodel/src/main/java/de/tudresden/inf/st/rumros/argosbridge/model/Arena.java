package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import de.tudresden.inf.st.rumros.argosbridge.ObjectUtils;

/**
 * This class is the Java XML object representation of the <arena> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Arena {

    /** The (x,y,z) size of the arena */
    @XmlAttribute
    private String size;

    /** The (x,y,z) center point of the arena */
    @XmlAttribute
    private String center;

    /** The entities contained in the arena */
    @XmlAnyElement(lax = true)
    private List<Object> entities = new ArrayList<>();

    /**
     * Gets the size of the arena.
     * @return the (x,y,z) size of the arena
     */
    public String getSize() { return size; }

    /**
     * Sets the size of the arena.
     * @param size the (x,y,z) size to set
     */
    public void setSize(String size) { this.size = size; }

    /**
     * Gets the center point of the arena.
     * @return the (x,y,z) center point
     */
    public String getCenter() { return center; }

    /**
     * Sets the center of the arena.
     * @param center the (x,y,z) center to set
     */
    public void setCenter(String center) { this.center = center; }

    /**
     * Gets the entities contained in the arena.
     * @return the list of entities
     */
    public List<Object> getEntities() { return entities; }

    /**
     * Sets the list of entities contained in the arena.
     * @param entities the list of entities
     */
    public void setEntities(List<Object> entities) { this.entities = entities; }

    /**
     * Dynamically adds a robot XML element (e.g. <foot-bot>) to the arena.
     * @param robotType the name of the robot tag (e.g., "foot-bot")
     * @param id the ID to assign to the robot
     * @param controllerId the controller config ID to use
     * @param position the position of the robot in the arena
     * @param orientation the direction the robot should face
     */
    public void addRobot(String robotType, String id, String controllerId, String position, String orientation) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
            Element robotElement = doc.createElement(robotType);
            robotElement.setAttribute("id", id);

            Element body = doc.createElement("body");
            body.setAttribute("position", position);
            body.setAttribute("orientation", orientation);

            Element controller = doc.createElement("controller");
            controller.setAttribute("config", controllerId);

            robotElement.appendChild(body);
            robotElement.appendChild(controller);

            this.entities.add(robotElement);
        } catch (Exception e) {
            throw new RuntimeException("An error occurred while trying to add <" + robotType + "> tag to Argos configuration", e);
        }
    }

    /**
     * Dynamically adds a robot XML element (e.g. <foot-bot>) to the arena.
     * @param robotType the name of the robot tag (e.g., "foot-bot")
     * @param id the ID to assign to the robot
     * @param groupId the ID of the group of the robot
     * @param controllerId the controller config ID to use
     * @param position the position of the robot in the arena
     * @param orientation the direction the robot should face
     */
    public void addRobot(String robotType, String id, String groupId, String controllerId, String position, String orientation) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
            Element robotElement = doc.createElement(robotType);
            robotElement.setAttribute("id", id);
            robotElement.setAttribute("group_id", groupId);

            Element body = doc.createElement("body");
            body.setAttribute("position", position);
            body.setAttribute("orientation", orientation);

            Element controller = doc.createElement("controller");
            controller.setAttribute("config", controllerId);

            robotElement.appendChild(body);
            robotElement.appendChild(controller);

            this.entities.add(robotElement);
        } catch (Exception e) {
            throw new RuntimeException("An error occurred while trying to add <" + robotType + "> tag to Argos configuration", e);
        }
    }

    /**
     * Adds a floor to the arena, which is a texture that can be defined via loop functions.
     * @param id the unique id of the floor
     * @param source the source texture for this floor, e.g. loop_functions
     * @param pixelsPerMeter the number of pixels the floor has per meter
     */
    public void addFloor(String id, String source, String pixelsPerMeter) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
            Element floorElement = doc.createElement("floor");
            floorElement.setAttribute("id", id);
            floorElement.setAttribute("source", source);
            floorElement.setAttribute("pixels_per_meter", pixelsPerMeter);

            this.entities.add(floorElement);
        } catch (Exception e) {
            throw new RuntimeException("An error occurred while trying to add <floor> tag to Argos configuration", e);
        }
    }

    /**
     * Distributes physical objects with varying orientations in the arena.
     * @param object the tag of object to distribute
     * @param quantity the number of clones to create
     * @param posMethod the method for distributing the objects in the arena
     * @param min the position string (x,y,z) of the minimum distribution position
     * @param max the position string (x,y,z) of the maximum distribution position
     * @param oriMethod the method for orienting the objects
     * @param mean the mean of the orientation distribution
     * @param stdDev the standard deviation of the orientation distribution
     * @param maxTrials the maximum number of trials to get a valid distribution
     */
    public void distributePhysicalObjects(Element object, int quantity,
                                          String posMethod, String min, String max,
                                          String oriMethod, String mean, String stdDev, 
                                          int maxTrials) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();

            Element distribute = doc.createElement("distribute");

            Element position = doc.createElement("position");
            position.setAttribute("method", posMethod);
            position.setAttribute("min", min);
            position.setAttribute("max", max);

            Element orientation = doc.createElement("orientation");
            orientation.setAttribute("method", oriMethod);
            orientation.setAttribute("mean", mean);
            orientation.setAttribute("std_dev", stdDev);

            Element entity = doc.createElement("entity");
            entity.setAttribute("quantity", String.valueOf(quantity));
            entity.setAttribute("max_trials", String.valueOf(maxTrials));

            ObjectUtils.importAndAddElement(entity, object);

            distribute.appendChild(position);
            distribute.appendChild(orientation);
            distribute.appendChild(entity);

            this.entities.add(distribute);
        } catch (Exception e) {
            throw new RuntimeException("Error adding distribute element: " + e.getMessage(), e);
        }
    }

    /**
     * Distributes physical objects with a constant orientation in the arena.
     * @param object the tag of object to distribute
     * @param quantity the number of clones to create
     * @param posMethod the method for distributing the objects in the arena
     * @param min the position string (x,y,z) of the minimum distribution position
     * @param max the position string (x,y,z) of the maximum distribution position
     * @param oriValues the values for object orientation
     * @param maxTrials the maximum number of trials to get a valid distribution
     */
    public void distributePhysicalObjects(Element object, int quantity,
                                          String posMethod, String min, String max,
                                          String oriValues, 
                                          int maxTrials) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();

            Element distribute = doc.createElement("distribute");

            Element position = doc.createElement("position");
            position.setAttribute("method", posMethod);
            position.setAttribute("min", min);
            position.setAttribute("max", max);

            Element orientation = doc.createElement("orientation");
            orientation.setAttribute("method", "constant");
            orientation.setAttribute("values", oriValues);

            Element entity = doc.createElement("entity");
            entity.setAttribute("quantity", String.valueOf(quantity));
            entity.setAttribute("max_trials", String.valueOf(maxTrials));

            ObjectUtils.importAndAddElement(entity, object);

            distribute.appendChild(position);
            distribute.appendChild(orientation);
            distribute.appendChild(entity);

            this.entities.add(distribute);
        } catch (Exception e) {
            throw new RuntimeException("Error adding distribute element: " + e.getMessage(), e);
        }
    }


    /**
     * Distributes robots of the given type in the arena.
     * @param robotType the name of the robot to distribute
     * @param robotIdPrefix the prefix for the individual robot IDs, e.g. "bot" for bot0, bot1, bot2, etc.
     * @param controllerId the controller config ID to use for the robots
     * @param quantity the number of robots to distribute
     * @param posMethod the method for distributing the objects in the arena
     * @param min the position string (x,y,z) of the minimum distribution position
     * @param max the position string (x,y,z) of the maximum distribution position
     */
    public void distributeRobots(String robotType, String robotIdPrefix, String controllerId,
                                 int quantity, String posMethod, String min, String max) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();

            Element robot = doc.createElement(robotType);
            robot.setAttribute("id", robotIdPrefix);

            Element controller = doc.createElement("controller");
            controller.setAttribute("config", controllerId);

            robot.appendChild(controller);

            distributePhysicalObjects(robot, quantity, posMethod, min, max, "gaussian", "0,0,0", "360,0,0", 100);
        } catch (Exception e) {
            throw new RuntimeException("Error adding distribute element: " + e.getMessage(), e);
        }
    }
}