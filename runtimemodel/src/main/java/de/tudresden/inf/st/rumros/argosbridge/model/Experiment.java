package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <experiment> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Experiment {

    /** The length of the experiment */
    @XmlAttribute
    private int length;

    /** The tick speed of the simulator */
    @XmlAttribute(name = "ticks_per_second")
    private int ticksPerSecond;

    /** The random seed of the simulator */
    @XmlAttribute(name = "random_seed")
    private int randomSeed;

    /**
     * Gets the length of the simulation.
     * @return the length of the simulation
     */
    public int getLength() { return length; }

    /**
     * Sets the length of the simulation.
     * @param length the length of the simulation to set
     */
    public void setLength(int length) { this.length = length; }

    /**
     * Gets the tick speed of the simulator.
     * @return the number of ticks per second
     */
    public int getTicksPerSecond() { return ticksPerSecond; }

    /**
     * Sets the tick speed of the simulator.
     * @param ticksPerSecond the number of ticks per second to set
     */
    public void setTicksPerSecond(int ticksPerSecond) { this.ticksPerSecond = ticksPerSecond; }

    /**
     * Gets the random seed of the simulator.
     * @return the random seed value
     */
    public int getRandomSeed() { return randomSeed; }

    /**
     * Sets the random seed of the simulator.
     * @param randomSeed the random seed value to set
     */
    public void setRandomSeed(int randomSeed) { this.randomSeed = randomSeed; }
}