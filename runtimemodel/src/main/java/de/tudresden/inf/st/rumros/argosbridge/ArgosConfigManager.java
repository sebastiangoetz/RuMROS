package de.tudresden.inf.st.rumros.argosbridge;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.io.FileNotFoundException;
import java.io.InputStream;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

import jakarta.xml.bind.*;
import de.tudresden.inf.st.rumros.argosbridge.model.*;
import de.tudresden.inf.st.rumros.runtimemodel.Model;

/**
 * This class contains utilities to map a scenario specification in JastAdd to a
 * configuration file for the ARGoS 3 simulator. It offers functions to initialize
 * objects and robots in the simulated space, as well as to load and modify templates
 * with pre-defined scenarios. This allows developers to flexibly define simulation
 * environments, into which a number of robots, specified in the runtime model scenario
 * specification is automatically inserted. 
 */
public class ArgosConfigManager {

    /** The folder containing ARGoS configuration templates */
    public static final String TEMPLATE_FOLDER = "argos-templates/";

    /** The parsed ARGoS XML configuration file */
    private ArgosConfiguration config;

    /** Creates the configuration manager object. */
    public ArgosConfigManager() {
        this.config = null;
    }

    /**
     * Creates the configuration manager object with a given configuration file.
     * @param xmlFile the configuration file to associate with this manager
     * @throws Exception if the given configuration file cannot be loaded
     */
    public ArgosConfigManager(File xmlFile) throws Exception {
        this.config = loadFile(xmlFile);
    }

    /**
     * Creates the configuration manager object with a configuration from the given {@link InputStream}.
     * @param xmlFileStream the input stream to read the configuration from
     * @throws Exception if the given configuration file cannot be loaded
     */
    public ArgosConfigManager(InputStream xmlFileStream) throws Exception {
        this.config = loadFile(xmlFileStream);
    }

    /**
     * Parses an XML configuration file for the ARGoS simulator into a Java object.
     * @param xmlFile the XML file to read the configuration from
     * @return the parsed configuration object
     * @throws JAXBException if the configuration file cannot be parsed
     */
    public ArgosConfiguration loadFile(File xmlFile) throws JAXBException {
        JAXBContext jaxbContext = JAXBContext.newInstance(ArgosConfiguration.class);
        Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();
        this.config = (ArgosConfiguration) unmarshaller.unmarshal(xmlFile);
        return config;
    }

    /**
     * Parses an XML configuration for the ARGoS simulator from a given {@link InputStream} into a Java object.
     * @param xmlFileStream the input stream to read the configuration from
     * @return the parsed configuration object
     * @throws JAXBException if the configuration file cannot be parsed
     */
    public ArgosConfiguration loadFile(InputStream xmlFileStream) throws JAXBException {
        JAXBContext jaxbContext = JAXBContext.newInstance(ArgosConfiguration.class);
        Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();
        this.config = (ArgosConfiguration) unmarshaller.unmarshal(xmlFileStream);
        return config;
    }

    /**
     * Checks whether the configuration pointer is valid. Can be extended in the future
     * to also check validity against an XML schema if required.
     * 
     * @return true if the configuration exists, false otherwise
     */
    private boolean checkConfiguration() {
        return (this.config == null) ? false : true;
    }

    /**
     * Returns the object representing the ARGoS configuration.
     * @return the ARGoS configuration object
     */
    public ArgosConfiguration getConfig() {
        return config;
    }

    /**
     * Creates a controller element in the XML configuration with the given name, identifier and implementation.
     * @param name the name of the controller tag
     * @param id the unique identifier of the controller
     * @param library the path to the library containing the code for the controller
     * @return the generated XML element
     */
    public Element createBasicController(String name, String id, String library) {
        return ObjectUtils.createElement(name, Map.of("id", id, "library", library));
    }

    /**
     * Adds a controller for the given robot type to the ARGoS configuration.
     * @param robotName the name/type of the robot, which has to match a folder name in resources/argos-templates
     * @param controllerName the name of the XML tag of the controller
     * @param controllerId the ID of the controller for later reuse
     * @param controllerLibrary the library of the controller
     */
    public void addRobotController(String robotName, String controllerName, String controllerId, String controllerLibrary) {
        if (!checkConfiguration())
            throw new IllegalStateException("Configuration is invalid, cannot add controller");

        // Add controller to <controllers>
        Element controller = createBasicController(controllerName, controllerId, controllerLibrary);

        // Load and add the appropriate robot configurations from the resource directory
        // To add support for more robots, templates should be added there accordingly
        try {
            DocumentBuilder dBuilder = DocumentBuilderFactory.newInstance().newDocumentBuilder();

            InputStream sensorsXml = ArgosConfigManager.class
            .getClassLoader()
            .getResourceAsStream(TEMPLATE_FOLDER + robotName + "/sensors.xml");

            InputStream actuatorsXml = ArgosConfigManager.class
            .getClassLoader()
            .getResourceAsStream(TEMPLATE_FOLDER + robotName + "/actuators.xml");

            InputStream paramsXml = ArgosConfigManager.class
            .getClassLoader()
            .getResourceAsStream(TEMPLATE_FOLDER + robotName + "/params.xml");

            Element sensors = dBuilder.parse(sensorsXml).getDocumentElement();
            Element actuators = dBuilder.parse(actuatorsXml).getDocumentElement();
            Element params = dBuilder.parse(paramsXml).getDocumentElement();

            ObjectUtils.importAndAddElement(controller, sensors);
            ObjectUtils.importAndAddElement(controller, actuators);
            ObjectUtils.importAndAddElement(controller, params);

            // Create empty controllers tag if it doesn't exist
            if (config.getControllers() == null) {
                config.setControllers(new Controllers());
                config.getControllers().setControllers(new ArrayList<>());
            }
            config.getControllers().getControllers().add(controller);
        } catch (Exception e) {
            throw new RuntimeException("Error adding distribute element: " + e.getMessage(), e);
        }
    }

    /**
     * Adds a ROS controller to the ARGoS configuration.
     * @param robotName the name/type of the robot, which has to match a folder name in resources/argos-templates
     */
    public String addRobotROSController(String robotName) {
        String libraryPath = "build/argos3_ros2_bridge/plugins/bridge/lib" + robotName + "_bridge";
        String controllerName = robotName + "_ros_controller";
        
        addRobotController(robotName, controllerName, controllerName, libraryPath);
        return controllerName;
    }

    /**
     * Adds a medium to the media tag.
     * @param elementName the name of the medium
     * @param elementId the id of the medium
     */
    public void addMedium(String elementName, String elementId) {
        if (!checkConfiguration())
            throw new IllegalStateException("Configuration is invalid, cannot add medium");

        // If there is no loop_functions tag, create one
        if (config.getMedia() == null) {
            config.setMedia(new Media());
            config.getMedia().setMedia(new ArrayList<>());
        }

        Element medium = ObjectUtils.createElement(elementName, Map.of("id", elementId));
        config.getMedia().getMedia().add(medium);
    }

    /**
     * Adds a physics engine to the physics_engines tag.
     * @param elementName the name of the engine
     * @param elementId the id of the engine
     * @param attributes a map containing further, object-specific attributes to add
     */
    public void addPhysicsEngine(String elementName, String elementId, Map<String, String> attributes) {
        if (!checkConfiguration())
            throw new IllegalStateException("Configuration is invalid, cannot add medium");

        // If there is no loop_functions tag, create one
        if (config.getPhysicsEngines() == null) {
            config.setPhysicsEngines(new PhysicsEngines());
            config.getPhysicsEngines().setEngines(new ArrayList<>());
        }
        if (attributes == null)
            attributes = new HashMap<>();
        else
            attributes = new HashMap<>(attributes); // attributes may be an immutable map, so create a copy
        attributes.put("id", elementId);
        Element engine = ObjectUtils.createElement(elementName, attributes);
        config.getPhysicsEngines().getEngines().add(engine);
    }

    /**
     * Adds loop functions to the ARGoS configuration.
     * @param loopFuncLabel the label for later reuse
     * @param loopFuncLibrary the library used to provide the actual functionality
     * @param content any contained child XML tags to add, which libraries can read in as parameters
     */
    public void addLoopFunctions(String loopFuncLabel, String loopFuncLibrary) {
        if (!checkConfiguration())
            throw new IllegalStateException("Configuration is invalid, cannot add loop functions");

        // If there is no loop_functions tag, create one
        if (config.getLoopFunctions() == null) {
            config.setLoopFunctions(new LoopFunctions()); 
        } else {
            throw new IllegalStateException("Configuration already contains loop_functions, cannot add more");
        }

        config.getLoopFunctions().setLabel(loopFuncLabel);
        config.getLoopFunctions().setLibrary(loopFuncLibrary);

        // If no content (parameters) exists, create it
        if (config.getLoopFunctions().getParameters() == null) {
            config.getLoopFunctions().setParameters(new ArrayList<>());
        } else {
            // Overwrote an existing loop_functions tag, clear content
            config.getLoopFunctions().getParameters().clear();
        }
    }

    /**
     * Adds QT user functions to the ARGoS configuration, which can be used to
     * draw on the simulation screen for instance.
     * @param userFuncLabel the label for later reuse
     * @param userFuncLibrary the library used to provide the actual functionality
     */
    public void addUserFunctions(String userFuncLabel, String userFuncLibrary) {
        if (!checkConfiguration())
            throw new IllegalStateException("Configuration is invalid, cannot add user functions");

        // If there is no user_functions tag, create one
        QtOpengl qtOpengl = config.getVisualization().getQtOpenGl();
        if (qtOpengl.getUserFunctions() == null) {
            qtOpengl.setUserFunctions(new UserFunctions());
        } else {
            throw new IllegalStateException("Configuration already contains user_functions, cannot add more");
        }

        qtOpengl.getUserFunctions().setLabel(userFuncLabel);
        qtOpengl.getUserFunctions().setLibrary(userFuncLibrary);
    }

    /**
     * Saves the current configuration to a file.
     * @param file the file to write to
     * @throws Exception if any errors are encountered writing the file
     */
    public void saveTo(File file) throws Exception {
        JAXBContext jaxbContext = JAXBContext.newInstance(ArgosConfiguration.class);
        Marshaller marshaller = jaxbContext.createMarshaller();
        marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);

        // Create DOM document
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.newDocument();
        marshaller.marshal(config, doc);

        // Pretty-print (2 space indentations)
        Transformer transformer = TransformerFactory.newInstance().newTransformer();
        transformer.setOutputProperty(OutputKeys.INDENT, "yes");
        transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");

        DOMSource source = new DOMSource(doc);
        StreamResult result = new StreamResult(file);
        transformer.transform(source, result);
    }

    /**
     * Creates the simulation according to selected templates and JastAdd extensions.
     * The first parameter is used to pass the path to the output file.
     * @param args the parameters to pass to the configuration manager
     * @throws Exception if any errors are encountered
     */
    public static void main(String[] args) throws Exception {
        //This will create the argos configuration for the project
        InputStream argosTemplate = ArgosConfigManager.class
            .getClassLoader()
            .getResourceAsStream(TEMPLATE_FOLDER + Model.ARGOS_TEMPLATE);

        if (argosTemplate == null) {
            throw new FileNotFoundException("Argos template not found in resources");
        }

        //Load config template
        ArgosConfigManager configManager = new ArgosConfigManager(argosTemplate);

        //Call JastAdd hook: modify template according to user specification
        //Init is called first and then the model is passed, so that
        //the setup method has access to all declarations made in the runtime model
        Model model = new Model().init();
        model.setupArgosSimulation(configManager, model);

        //Save config file
        configManager.saveTo(new File(args[0]));
    }
}
