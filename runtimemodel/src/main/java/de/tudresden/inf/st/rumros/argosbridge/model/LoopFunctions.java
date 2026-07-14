package de.tudresden.inf.st.rumros.argosbridge.model;
import de.tudresden.inf.st.rumros.argosbridge.ObjectUtils;

import jakarta.xml.bind.annotation.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.w3c.dom.Element;

/**
 * This class is the Java XML object representation of the <loop_functions> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class LoopFunctions {

    /** The generic parameters of the loop functions */
    @XmlAnyElement(lax = true)
    private List<Element> parameters = new ArrayList<>();

    /** The path of the library with implementation code */
    @XmlAttribute
    private String library;

    /** The label of the loop functions */
    @XmlAttribute
    private String label;

    /**
     * Gets the parameters of the loop functions.
     * @return the parameter list
     */
    public List<Element> getParameters() { return parameters; }

    /**
     * Sets the parameters of the loop functions.
     * @param parameters the parameter list to set
     */
    public void setParameters(List<Element> parameters) { this.parameters = parameters; }

    /**
     * Gets the path to the library with implementation code.
     * @return the path to the library
     */
    public String getLibrary() { return library; }

    /**
     * Sets the path to the library with implementation code.
     * @param library the path to set
     */
    public void setLibrary(String library) { this.library = library; }

    /**
     * Gets the label of the loop functions.
     * @return the label
     */
    public String getLabel() { return label; }

    /**
     * Sets the label of the loop functions.
     * @param label the label to set
     */
    public void setLabel(String label) { this.label = label; }

    /**
     * Gets the <options> tag from the associated parameters.
     * @return the options tag or null if it doesn't exist
     */
    public Element getOptions() {
        for (Element e : parameters) {
            if (e.getTagName() == "options")
                return e;
        }
        return null;
    }

    /**
     * Gets the <areas> tag from the associated parameters.
     * @return the areas tag or null if it doesn't exist
     */
    public Element getAreaOptions() {
                for (Element e : parameters) {
            if (e.getTagName() == "areas")
                return e;
        }
        return null;
    }

    /**
     * Adds an area option to the <areas> tag in the associated parameters.
     * @param area the area to add, with name and attributes x1, x2, y1 and y2
     */
    public void addAreaOption(Map<String, String> area) {
        // For now, only boolean options are supported
        Element eAreas = getAreaOptions();
        if (eAreas == null) {
            eAreas = ObjectUtils.createElement("areas", Map.of());
            parameters.add(eAreas);
        }
            
        // Create and import area element
        Element eArea = ObjectUtils.createElement("area", area);
        ObjectUtils.importAndAddElement(eAreas, eArea);
    }

    /**
     * Adds boolean options to the <options> tag in the associated parameters.
     * @param options the options to add as key-value pairs
     */
    public void addBooleanOptions(Map<String, Boolean> options) {
        // For now, only boolean options are supported
        Element eOptions = getOptions();
        if (eOptions == null) {
            eOptions = ObjectUtils.createElement("options", Map.of());
            parameters.add(eOptions);
        }
            
        for (Map.Entry<String, Boolean> e : options.entrySet()) {
            // Create element: <key enabled=val/>
            Element eOpt = ObjectUtils.createElement(e.getKey(), Map.of("enabled", e.getValue().toString()));
            ObjectUtils.importAndAddElement(eOptions, eOpt);
        }
    }
}
