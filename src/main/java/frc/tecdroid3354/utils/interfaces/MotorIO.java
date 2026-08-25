package frc.tecdroid3354.utils.interfaces;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface MotorIO {

    /**
     * For motors to have their own inputs object, allowing subsystem input objects to focus in subsystem-wise inputs.
     * Note that you will need to process this inputs separately inside your subsystem periodic() method.
     */
    @AutoLog
    class MotorIOInputs {
        public boolean isConnected = false;

        public MutAngle closedLoopReference = Degrees.mutable(0.0);
        public MutAngularVelocity closedLoopReferenceSlope = DegreesPerSecond.mutable(0.0);

        public MutAngle currentPosition = Degrees.mutable(0.0);
        public MutAngularVelocity currentVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularAcceleration currentAcceleration = DegreesPerSecondPerSecond.mutable(0.0);

        public MutTemperature currentTemperature = Celsius.mutable(0.0);
        public MutVoltage outputVoltage = Volts.mutable(0.0);
        public MutCurrent supplyCurrent = Amps.mutable(0.0);
        public MutCurrent torqueCurrent = Amps.mutable(0.0);
    }

    default void updateInputs(MotorIOInputs inputs) {}
}
