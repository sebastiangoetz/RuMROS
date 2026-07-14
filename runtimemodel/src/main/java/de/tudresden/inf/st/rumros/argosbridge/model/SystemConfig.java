package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <system> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class SystemConfig {

    /**
     * The number of threads to use for simulation
     */
    @XmlAttribute
    private int threads;

    /**
     * Gets the number of threads for simulation.
     * @return the number of threads
     */
    public int getThreads() { return threads; }

    /**
     * Sets the number of threads for simulation.
     * @param threads the number of threads to set
     */
    public void setThreads(int threads) { this.threads = threads; }
}