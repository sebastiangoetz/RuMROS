package de.tudresden.inf.st.rumros.runtimemodel;

/** Auxiliary class that simplifies initialization code for empty result types. */
public class EmptyResult extends Result {

    /** Creates the empty result object. */
    public EmptyResult() {
        super(ActionResultType.EMPTY, null, 0);
    }
}
