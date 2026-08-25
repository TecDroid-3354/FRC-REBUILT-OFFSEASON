package frc.tecdroid3354.subsystems.angularVelocity;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link AngularVelocity} driven subsystem, i.e. a Flywheel.
 * Duplicate the motor-related fields inside {@link FlywheelIOInputs} for the number of motors in your subsystem,
 * or delete the follower motor fields if your subsystem is single-motor.
 * <p>
 * Any changes to this or any template present in the repository must be discussed with the area Lead(s) and Captain.
 * </p>
 * <p>
 * Why is this file in Java if our primary language is Kotlin?
 *  <p>
 *  - Java annotations (i.e. {@link org.littletonrobotics.junction.AutoLog @AutoLog}) are not present in kotlin.
 *      This is solvable using kapt, yet having this sort of bridge can make builds / deploys take significantly longer.
 *      Note that kapt is still being used, yet only to read the {@link org.littletonrobotics.junction.AutoLog @AutoLog}
 *      generated files in kotlin classes, not to create them.
 *  </p>
 *  <p>
 *  - When using kapt to generate classes, we have to mark every field of the {@link FlywheelIOInputs}
 *      as @JvmField, which is just annoying and impacts readability.
 *  </p>
 * </p>
 */
public interface FlywheelIO {
    /**
     * Class intended to log all relevant fields that might change during a match.
     * These inputs may be used for:
     *  <p>- Creating alerts (i.e. when a motor disconnects)</p>
     *  <p>- Tune PIDF constants in AdvantageScope (using Mechanical Advantage's LoggedTunableNumber)</p>
     *  <p>- Replay a match in AdvantageScope (using Replay Mode)</p>
     *
     *  All of these fields should be published periodically through
     *  {@link org.littletonrobotics.junction.Logger Logger.processInputs(String, IOInputsAutoLogged)}
     *
     *  <p>
     *      If you are having trouble with your generated {@link FlywheelIOInputsAutoLogged}, it is most probably
     *      because of wrong kapt configuration. Ask an Area Lead or Captain.
     *  </p>
     */
    @AutoLog
    class FlywheelIOInputs {
        /** Flywheel wise fields */
        public MutAngularVelocity flywheelActualVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelManualTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelPresetVelocity = DegreesPerSecond.mutable(0.0); // If applicable
    }

    /**
     * Intended to update any relevant fields in {@link FlywheelIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link FlywheelIOInputs} object keeping track of everything.
     */
    void updateFlywheelInputs(FlywheelIOInputs inputs,
                              MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs);

    /**
     * Used to update the in-file variable containing the manual target velocity. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newFlywheelManualVelocity Obtained live through Elastic.
     */
    void updateFlywheelManualVelocity(AngularVelocity newFlywheelManualVelocity);

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    void updateFlywheelMotorsControlGains(int slot);

    /**
     * Sets the subsystem to the manually set velocity through Elastic. This resets with every code reload.
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @see #updateFlywheelManualVelocity(AngularVelocity)
     * @return A {@link Runnable} setting the subsystem manual target velocity
     */
    Runnable enableFlywheelManualVelocity();

    /**
     * Only if applicable.
     * <p>Sets the subsystem to the preset velocity stored in constants.</p>
     * <p>This does not change live, only in-code.</p>
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @return A {@link Runnable} setting the subsystem preset target velocity
     */
    Runnable enableFlywheelPresetVelocity();

    /**
     * Only if applicable. For distance based / state based subsystems specifically.
     * <p>Sets the calculated subsystem velocity, i.e. through interpolation / polynomial</p>
     * <p>Calculation is delegated to a separate method</p>
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @param flywheelCalculatedVelocity Flywheel velocity; must be calculated beforehand.
     * @return A {@link Runnable} setting the subsystem calculated target velocity
     */
    Runnable enableFlywheelCalculatedVelocity(AngularVelocity flywheelCalculatedVelocity);

    /**
     * Disables the subsystem motors.
     * @return A {@link Runnable} stopping the subsystem
     */
    Runnable stopFlywheel();

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     * @return A {@link Runnable} coasting all subsystem motors
     */
    Runnable coastFlywheelMotors();

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     * @return A {@link Runnable} braking all subsystem motors
     */
    Runnable brakeFlywheelMotors();

    /**
     * Applies the configuration inside {@link FlywheelConstants.PhoenixMotorConfiguration}. Follower commands are included.
     */
    void initialMotorConfiguration();

    /**
     * **IMPORTANT:** This class is meant to use in REPLAY mode only. It leaves every method empty,
     * since REPLAY mode only recreates what happened and does not need to process anything through those methods.
     *
     * <p>It is created to avoid doing this inside {@link frc.tecdroid3354.core.RobotContainer} and to avoid
     * having to give every method a default, empty implementation since it would prevent notifications to override
     * them in other layers.
     * </p>
     */
    class DummyFlywheelIO implements FlywheelIO {

        @Override
        public void updateFlywheelInputs(FlywheelIOInputs inputs,
                                         MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {

        }

        @Override
        public void updateFlywheelManualVelocity(AngularVelocity newFlywheelManualVelocity) {

        }

        @Override
        public void updateFlywheelMotorsControlGains(int slot) {

        }

        @Override
        public Runnable enableFlywheelManualVelocity() {
            return null;
        }

        @Override
        public Runnable enableFlywheelPresetVelocity() {
            return null;
        }

        @Override
        public Runnable enableFlywheelCalculatedVelocity(AngularVelocity flywheelCalculatedVelocity) {
            return null;
        }

        @Override
        public Runnable stopFlywheel() {
            return null;
        }

        @Override
        public Runnable coastFlywheelMotors() {
            return null;
        }

        @Override
        public Runnable brakeFlywheelMotors() {
            return null;
        }

        @Override
        public void initialMotorConfiguration() {

        }
    }
}
