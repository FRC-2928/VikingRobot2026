package frc.robot.utils;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.stream.Stream;

import com.ctre.phoenix6.BaseStatusSignal;

public class SignalBundle<T> {
    private final BaseStatusSignal[] mSignals;
    private final Consumer<T> mUpdater;

    public SignalBundle(BaseStatusSignal[] signals, Consumer<T> updater) {
        this.mSignals = signals;
        this.mUpdater = updater;
    }

    public BaseStatusSignal[] signals() {
        return mSignals;
    }

    public void update(T inputs) {
        mUpdater.accept(inputs);
    }

    @SafeVarargs
    public static <T> BaseStatusSignal[] collectSignals(SignalBundle<T>... bundles) {
        return Arrays.stream(bundles)
            .flatMap(b -> Stream.of(b.signals()))
            .toArray(BaseStatusSignal[]::new);
    }
}
