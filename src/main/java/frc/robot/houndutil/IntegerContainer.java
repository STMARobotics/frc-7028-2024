package com.techhounds.houndutil.houndlib;

/**
 * A container for an integer value, used for passing by reference into a lambda
 * expression. see: {@link com.techhounds.houndutil.houndlib.leds.LEDPatterns}
 */
public class IntegerContainer {
    public int value;

    public IntegerContainer(int value) {
        this.value = value;
    }
}
