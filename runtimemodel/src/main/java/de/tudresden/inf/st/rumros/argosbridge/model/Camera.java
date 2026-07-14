package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <camera> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Camera {

    /** The associated placements of cameras */
    @XmlElement
    private Placements placements;

    /**
     * Gets the associated camera placements.
     * @return the list of placement objects
     */
    public Placements getPlacements() {
        return placements;
    }

    /**
     * Sets the associated camera placements.
     * @param placements the list of placement objects to set
     */
    public void setPlacements(Placements placements) {
        this.placements = placements;
    }
}
