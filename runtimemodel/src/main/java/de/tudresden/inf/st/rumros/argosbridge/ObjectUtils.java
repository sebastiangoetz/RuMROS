package de.tudresden.inf.st.rumros.argosbridge;

import java.util.Map;

import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * This class offers methods to create XML objects in an ARGoS configuration.
 * It is intended as a proof of concept, and therefore does not yet support all
 * object types which ARGoS supports. Extend with methods for new objects as
 * needed, by creating new methods that call createElement().
 */
public final class ObjectUtils {
    
    /**
     * Creates a cylinder in the simulation
     * @param id the unique id of the element
     * @param height the height of the cylinder
     * @param radius the radius of the cylinder
     * @param movable whether the cylinder is movable or not
     * @return
     */
    public static Element createCylinder(String id, String height, String radius, String movable) {
        return createElement("cylinder", Map.of("id", id, "height", height, "radius", radius, "movable", movable));
    }

    /**
     * Imports an element into another elements document and adds it as a child node.
     * @param parent the parent element p in document d1
     * @param newChild the element c in document d2 to import into d1 and add as a new child of p
     */
    public static void importAndAddElement(Element parent, Element newChild) {
        Document parentDoc = parent.getOwnerDocument();
        parent.appendChild(parentDoc.importNode(newChild, true));
    }

    /**
     * Creates an XML element with the given name and attributes.
     * @param name the name of the element
     * @param attributes a map of named string attributes and corresponding values
     * @return the created XML element
     */
    public static Element createElement(String name, Map<String, String> attributes) {
        try {
            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();

            Element element = doc.createElement(name);

            if (attributes != null) {
                for (Map.Entry<String, String> entry : attributes.entrySet()) {
                    element.setAttribute(entry.getKey(), entry.getValue());
                }
            }

            return element;
        } catch (Exception e) {
            throw new RuntimeException("Error creating <" + name + "> element: " + e.getMessage(), e);
        }
    }
}
