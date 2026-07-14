package de.tudresden.inf.st.rumros.argosbridge.model;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <physics_engines> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class PhysicsEngines {

    /** The list of associated physics engines */
    @XmlAnyElement(lax = true)
    private List<Object> engines = new ArrayList<>();

    /**
     * Gets the list of associated physics engines.
     * @return the list of physics engine objects
     */
    public List<Object> getEngines() {
        return engines;
    }

    /**
     * Sets the list of associated physics engines.
     * @param engines the list of physics engine objects to set
     */
    public void setEngines(List<Object> engines) {
        this.engines = engines;
    }
}