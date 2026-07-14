package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <user_functions> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class UserFunctions {

    /** The library with implementation code for the user functions */
    @XmlAttribute
    private String library;

    /** The label of the user functions */
    @XmlAttribute
    private String label;

    /**
     * Gets the implementation library path.
     * @return the path to the library
     */
    public String getLibrary() { return library; }

    /**
     * Sets the implementation library path.
     * @param library the path to the library to set
     */
    public void setLibrary(String library) { this.library = library; }

    /**
     * Gets the label of the user functions.
     * @return the label of the user functions
     */
    public String getLabel() { return label; }

    /**
     * Sets the label of the user functions.
     * @param label the label to set
     */
    public void setLabel(String label) { this.label = label; }

    
}
