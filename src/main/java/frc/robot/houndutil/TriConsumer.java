package com.techhounds.houndutil.houndlib;

import java.util.Objects;
import java.util.function.BiConsumer;

/**
 * Represents an operation that accepts three input arguments and returns no
 * result. This is the three-arity specialization of {@link Consumer}.
 * {@link BiConsumer} is a two-arity specialization.
 * 
 * <p>
 * 
 * Used wherever a function that takes in three parameters is necessary.
 */
@FunctionalInterface
public interface TriConsumer<A, B, C> {
    void accept(A a, B b, C c);

    default TriConsumer<A, B, C> andThen(TriConsumer<? super A, ? super B, ? super C> after) {
        Objects.requireNonNull(after);

        return (a, b, c) -> {
            accept(a, b, c);
            after.accept(a, b, c);
        };
    }
}
