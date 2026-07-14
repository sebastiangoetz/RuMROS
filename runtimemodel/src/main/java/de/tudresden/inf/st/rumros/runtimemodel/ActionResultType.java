package de.tudresden.inf.st.rumros.runtimemodel;

/**
 * Represents the outcome of an {@link Action} for display in the web UI.
 *
 * <p>Each result type is associated with a numeric identifier that can be
 * mapped in the frontend.</p>
 */
public enum ActionResultType {

    /**
     * No result is available.
     */
    EMPTY(-1),

    /**
     * The action completed successfully.
     */
    SUCCESS(0),

    /**
     * The action failed due to a functional or validation error.
     */
    ERROR(1),

    /**
     * The action failed due to an unexpected technical issue.
     */
    TECHNICAL_ERROR(2),

    /**
     * The action is still in progress.
     *
     * <p>This value is currently only used by {@link AwaitableResult}.</p>
     */
    WAIT(3);

    /**
     * Numeric identifier of the result type.
     */
    private final int id;

    /**
     * Creates a new action result type with the given identifier.
     * @param id numeric identifier associated with the result type
     */
    ActionResultType(int id) {
        this.id = id;
    }

    /**
     * Returns the numeric identifier of this result type.
     * @return the numeric result identifier
     */
    public int getId() {
        return id;
    }
}
