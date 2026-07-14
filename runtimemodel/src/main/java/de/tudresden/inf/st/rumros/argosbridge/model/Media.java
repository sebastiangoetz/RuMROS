package de.tudresden.inf.st.rumros.argosbridge.model;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <media> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Media {
    /** The list of media associated with this tag */
    @XmlAnyElement(lax = true)
    private List<Object> media = new ArrayList<>();

    /**
     * Gets the associated media.
     * @return the list of medium objects
     */
    public List<Object> getMedia() { return media; }

    /**
     * Sets the associated media.
     * @param media the list of media to set
     */
    public void setMedia(List<Object> media) { this.media = media; }
}
