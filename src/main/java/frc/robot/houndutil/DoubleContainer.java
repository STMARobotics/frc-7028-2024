package com.techhounds.houndutil.houndlib;

/**
 * A container for a double value, used for passing by reference into a lambda
 * expression. see: {@link com.techhounds.houndutil.houndlib.leds.LEDPatterns}
 */
public class DoubleContainer {
    public double value;

    public DoubleContainer(double value) {
        this.value = value;
    }
}
