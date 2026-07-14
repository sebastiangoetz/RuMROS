package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <placement> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Placement {

    /** The index of the placement in the parent list */
    @XmlAttribute
    private int index;

    /** The (x,y,z) position of the placement */
    @XmlAttribute
    private String position;

    /** The coordinate the camera associated with this placement looks at */
    @XmlAttribute(name = "look_at")
    private String lookAt;

    /** The up vector of the placement */
    @XmlAttribute
    private String up;

    /** The focal length of the camera associated with this placement in mm */
    @XmlAttribute(name = "lens_focal_length")
    private String lensFocalLength;   

    /**
     * Gets the index of the placement in the parent list.
     * @return the index
     */
    public int getIndex() {
        return index;
    }

    /**
     * Sets the index of the placement in the parent list.
     * @param index the index to set
     */
    public void setIndex(int index) {
        this.index = index;
    }

    /**
     * Gets the position of the placement.
     * @return the (x,y,z) position
     */
    public String getPosition() {
        return position;
    }

    /**
     * Sets the position of the placement.
     * @param position the (x,y,z) position to set
     */
    public void setPosition(String position) {
        this.position = position;
    }

    /**
     * Gets the look_at position of the associated camera.
     * @return the (x,y,z) position
     */
    public String getLookAt() {
        return lookAt;
    }

    /**
     * Sets the look_at position of the associated camera.
     * @param lookAt the (x,y,z) position to set
     */
    public void setLookAt(String lookAt) {
        this.lookAt = lookAt;
    }

    /**
     * Gets the up vector of the placement.
     * @return the (x,y,z) vector
     */
    public String getUp() {
        return up;
    }

    /**
     * Sets the up vector of the placement.
     * @param up the (x,y,z) vector to set
     */
    public void setUp(String up) {
        this.up = up;
    }

    /**
     * Gets the focal length of the associated camera.
     * @return the focal length in mm
     */
    public String getLensFocalLength() {
        return lensFocalLength;
    }

    /**
     * Sets the focal length of the associated camera.
     * @param lensFocalLength the focal length in mm to set
     */
    public void setLensFocalLength(String lensFocalLength) {
        this.lensFocalLength = lensFocalLength;
    }
}
