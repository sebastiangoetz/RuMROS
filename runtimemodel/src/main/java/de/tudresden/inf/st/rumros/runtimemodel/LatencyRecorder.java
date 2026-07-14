package de.tudresden.inf.st.rumros.runtimemodel;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import sun.misc.Signal;

/**
 * Auxiliary class for measuring and recording method execution latencies.
 *
 * Usage pattern:
 *   long t0 = LatencyRecorder.start();
 *   <code under measurement>
 *   LatencyRecorder.record(t0);
 *
 * At shutdown (or end of experiment):
 *   LatencyRecorder.writeToFile();
 */
public final class LatencyRecorder {

    /** Output file path */
    private static Path outputFile;

    /** Collected latencies in nanoseconds */
    private static final List<Long> latenciesNs = new ArrayList<>();

    /** Prevent instantiation */
    private LatencyRecorder() {}

    /** Initializes output file and signal handlers for safe termination. */
    public static void initialize() {
        setOutputFile("../analysis_results/pose_latencies.txt");

        // Write results on shutdown
        Signal.handle(new Signal("TERM"), sig -> {
            LatencyRecorder.flushSafely();
            System.exit(0);
        });

        Signal.handle(new Signal("INT"), sig -> {
            LatencyRecorder.flushSafely();
            System.exit(0);
        });

        System.out.println("Registered LatencyRecorder hooks");
        System.out.println("Writing to " + outputFile.toAbsolutePath());
    }



    /**
     * Sets the output file.
     * @param filePath path to the file
     */
    public static synchronized void setOutputFile(String filePath) {
        outputFile = Path.of(filePath);
    }

    /**
     * Starts the internal timer.
     * @return the current time in nanoseconds
     */
    public static long start() {
        return System.nanoTime();
    }

    /**
     * Records the latency from the given start to now in nanoseconds.
     * @param startNs start timestamp in nanoseconds
     */
    public static void record(long startNs) {
        long durationNs = System.nanoTime() - startNs;
        synchronized (latenciesNs) {
            latenciesNs.add(durationNs);
        }
    }

    /**
     * Writes all recorded latencies to the output file, one measurement in nanoseconds per line.
     * @throws IOException if the output file cannot be written
     */
    public static synchronized void writeToFile() throws IOException {
        System.out.println("Writing Latency results to " + outputFile.toAbsolutePath());
        if (outputFile == null) {
            throw new IllegalStateException("Output file not set. Call setOutputFile() first.");
        }

        try (BufferedWriter writer = Files.newBufferedWriter(
                outputFile,
                StandardOpenOption.CREATE,
                StandardOpenOption.TRUNCATE_EXISTING
        )) {
            synchronized (latenciesNs) {
                for (long ns : latenciesNs) {
                    writer.write(Long.toString(ns));
                    writer.newLine();
                }
            }
        }
    }

    /** Writes all latency data from memory to the output file, printing any occurring errors. */
    public static synchronized void flushSafely() {
        try {
            writeToFile();
        } catch (IOException e) {
            System.err.println("Failed to write latency data:");
            e.printStackTrace();
        }
    }

    /** Clears the internal recorded latency buffer. */
    public static void reset() {
        synchronized (latenciesNs) {
            latenciesNs.clear();
        }
    }

    /**
     * Counts the number of recorded samples.
     * @return the number of samples
     */
    public static int sampleCount() {
        synchronized (latenciesNs) {
            return latenciesNs.size();
        }
    }
}

