package de.tudresden.inf.st.rumros.argosbridge.model;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <placements> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Placements {

    /** The list of placements associated with this tag */
    @XmlElement(name = "placement")
    private List<Placement> placements = new ArrayList<>();

    /**
     * Gets the list of placements associated with this tag.
     * @return the list of placements
     */
    public List<Placement> getPlacements() {
        return placements;
    }

    /**
     * Sets the list of placements associated with this tag.
     * @param placements the list of placements to set
     */
    public void setPlacements(List<Placement> placements) {
        this.placements = placements;
    }
}
