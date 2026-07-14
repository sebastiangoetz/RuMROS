package de.tudresden.inf.st.rumros.argosbridge.model;

import jakarta.xml.bind.annotation.*;

/**
 * This class is the Java XML object representation of the <qt-opengl> configuration tag.
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class QtOpengl {

    /** The camera object associated with this tag */
    @XmlElement
    private Camera camera;

    /** The user functions associated with this tag */
    @XmlElement(name = "user_functions")
    private UserFunctions userFunctions;

    /**
     * Gets the camera object associated with this tag.
     * @return the camera object
     */
    public Camera getCamera() { return camera; }

    /**
     * Sets the camera object associated with this tag.
     * @param camera the camera object to set
     */
    public void setCamera(Camera camera) { this.camera = camera; }

    /**
     * Gets the QT user functions associated with this tag.
     * @return the QT user functions
     */
    public UserFunctions getUserFunctions() { return userFunctions; }

    /**
     * Sets the user functions associated with this tag.
     * @param userFunctions the QT user functions to set
     */
    public void setUserFunctions(UserFunctions userFunctions) { this.userFunctions = userFunctions; }
}
