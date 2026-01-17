package com.techhounds.houndutil.houndlib;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Class that tracks the position of multiple mechanisms on a robot. Used for
 * inter-mechanism safeties, or for when any tracking is needed between
 * subsystems.
 */
public class PositionTracker {
    private Map<String, Supplier<Double>> positionSuppliers = new HashMap<>();

    /**
     * Adds a mechanism position supplier to the tracker.
     * 
     * @param name             the name of the mechanism to track
     * @param positionSupplier the supplier of its position
     */
    public void addPositionSupplier(String name, Supplier<Double> positionSupplier) {
        positionSuppliers.put(name, positionSupplier);
    }

    /**
     * Gets the position of a mechanism.
     * 
     * @param name the name of the mechanism
     * @return the position of the mechanism
     */
    public double getPosition(String name) {
        Supplier<Double> supp = positionSuppliers.get(name);
        if (supp == null) {
            return 0;
        } else {
            return supp.get();
        }
    }
}
