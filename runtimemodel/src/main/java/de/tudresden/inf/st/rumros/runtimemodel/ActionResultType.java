package de.tudresden.inf.st.rumros.runtimemodel;

public enum ActionResultType {
    EMPTY(-1),
    SUCCESS(0),
    ERROR(1),
    TECHNICAL_ERROR(2);

    private final int id;

    ActionResultType(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
