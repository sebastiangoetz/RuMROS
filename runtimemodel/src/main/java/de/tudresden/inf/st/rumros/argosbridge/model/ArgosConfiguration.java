package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the root configuration tag.
 */
@XmlRootElement(name = "argos-configuration")
@XmlAccessorType(XmlAccessType.FIELD)
public class ArgosConfiguration {

    /** The associated framework object */
    @XmlElement
    private Framework framework;

    /** The associated controllers object */
    @XmlElement
    private Controllers controllers;

    /** The associated loop functions object */
    @XmlElement(name = "loop_functions")
    private LoopFunctions loopFunctions;

    /** The associated arena object */
    @XmlElement
    private Arena arena;

    /** The associated media object */
    @XmlElement
    private Media media;

    /** The associated physics engines objects */
    @XmlElement(name = "physics_engines")
    private PhysicsEngines physicsEngines;

    /** The associated visualization object */
    @XmlElement
    private Visualization visualization;

    /**
     * Gets the associated framework.
     * @return the framework object
     */
    public Framework getFramework() { return framework; }

    /**
     * Sets the associated framework.
     * @param framework the framework object to set
     */
    public void setFramework(Framework framework) { this.framework = framework; }

    /**
     * Gets the associated controllers.
     * @return the controllers object
     */
    public Controllers getControllers() { return controllers; }

    /**
     * Sets the associated controllers.
     * @param controllers the controllers object to set
     */
    public void setControllers(Controllers controllers) { this.controllers = controllers; }

    /**
     * Gets the associated loop functions.
     * @return the loop functions object
     */
    public LoopFunctions getLoopFunctions() { return loopFunctions; }

    /**
     * Sets the associated loop functions.
     * @param loopFunctions the loop functions object to set
     */
    public void setLoopFunctions(LoopFunctions loopFunctions) { this.loopFunctions = loopFunctions; }

    /**
     * Gets the associated arena.
     * @return the arena object
     */
    public Arena getArena() { return arena; }

    /**
     * Sets the associated arena.
     * @param arena the arena object to set
     */
    public void setArena(Arena arena) { this.arena = arena; }

    /**
     * Gets the associated media.
     * @return the media object
     */
    public Media getMedia() { return media; }

    /**
     * Sets the associated media.
     * @param media the media object to set
     */
    public void setMedia(Media media) { this.media = media; }

    /**
     * Gets the associated physics engines.
     * @return the physics engines object
     */
    public PhysicsEngines getPhysicsEngines() { return physicsEngines; }

    /**
     * Sets the associated physics engines.
     * @param physicsEngines the physics engines object to set
     */
    public void setPhysicsEngines(PhysicsEngines physicsEngines) { this.physicsEngines = physicsEngines; }

    /**
     * Gets the associated visualization.
     * @return the visualization object
     */
    public Visualization getVisualization() { return visualization; }

   /**
     * Sets the associated visualization.
     * @param visualization the visualization object to set
     */
    public void setVisualization(Visualization visualization) { this.visualization = visualization; }
}