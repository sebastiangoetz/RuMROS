package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <framework> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Framework {

    /** The associated system object */
    @XmlElement
    private SystemConfig system;

    /** The associated experiment object */
    @XmlElement
    private Experiment experiment;

    /**
     * Gets the associated system object.
     * @return the system object
     */
    public SystemConfig getSystem() { return system; }

    /**
     * Sets the associated system object.
     * @param system the system object to set
     */
    public void setSystem(SystemConfig system) { this.system = system; }

    /**
     * Gets the associated experiment object.
     * @return the experiment object
     */
    public Experiment getExperiment() { return experiment; }

    /**
     * Sets the associated experiment object.
     * @param experiment the experiment object to set
     */
    public void setExperiment(Experiment experiment) { this.experiment = experiment; }
}
