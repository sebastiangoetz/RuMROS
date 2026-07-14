package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is the Java XML object representation of the <controllers> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Controllers {

    /** The list of controllers associated with this tag */
    @XmlAnyElement(lax = true)
    private List<Object> controllers = new ArrayList<>();

    /**
     * Gets the associated controllers.
     * @return the list of controller objects
     */
    public List<Object> getControllers() { return controllers; }

    /**
     * Sets the associated controllers.
     * @param controllers the list of controllers to set
     */
    public void setControllers(List<Object> controllers) { this.controllers = controllers; }
}