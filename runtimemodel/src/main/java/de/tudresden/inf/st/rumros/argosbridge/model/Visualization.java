package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <visualization> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Visualization {

    /** The contained qt-opengl object */
    @XmlElement(name = "qt-opengl")
    private QtOpengl qtOpenGl;

    /**
     * Gets the qt-opengl object.
     * @return the qt-opengl object
     */
    public QtOpengl getQtOpenGl() {
        return qtOpenGl;
    }

    /**
     * Sets the qt-opengl object.
     * @param qtOpenGl the qt-opengl object to set
     */
    public void setQtOpenGl(QtOpengl qtOpenGl) {
        this.qtOpenGl = qtOpenGl;
    }
}