package frc.tecdroid3354.subsystems.Hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.DegreesPerSecond;

/**
 * I/O (Input/Output) interface intended for any {@link AngularVelocity} driven subsystem, i.e. a Flywheel.
 * Duplicate the motor-related fields inside {@link HopperIOInputs} for the number of motors in your subsystem,
 * or delete the follower motor fields if your subsystem is single-motor.
 * <p>
 * Any changes to this or any template present in the repository must be discussed with the area Lead(s) and Captain.
 * </p>
 * <p>
 * Why is this file in Java if our primary language is Kotlin?
 *  <p>
 *  - Java annotations (i.e. {@link AutoLog @AutoLog}) are not present in kotlin.
 *      This is solvable using kapt, yet having this sort of bridge can make builds / deploys take significantly longer.
 *      Note that kapt is still being used, yet only to read the {@link AutoLog @AutoLog}
 *      generated files in kotlin classes, not to create them.
 *  </p>
 *  <p>
 *  - When using kapt to generate classes, we have to mark every field of the {@link HopperIOInputs}
 *      as @JvmField, which is just annoying and impacts readability.
 *  </p>
 * </p>
 */
public interface HopperIO {
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
     *      If you are having trouble with your generated {@link HopperIOInputsAutoLogged}, it is most probably
     *      because of wrong kapt configuration. Ask an Area Lead or Captain.
     *  </p>
     */
    @AutoLog
    class HopperIOInputs {
        /** Subsystem wise fields */
        public MutAngularVelocity hopperActualVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity hopperTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity hopperManualTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity hopperPresetVelocity = DegreesPerSecond.mutable(0.0); // If applicable
    }

    /**
     * Intended to update any relevant fields in {@link HopperIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link HopperIOInputs} object keeping track of everything.
     */
    void updateHopperInputs(HopperIOInputs inputs,
                            MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs);

    /**
     * Used to update the in-file variable containing the manual target velocity. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newHopperManualVelocity Obtained live through Elastic.
     */
    void updateHopperManualVelocity(AngularVelocity newHopperManualVelocity);

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    void updateHopperMotorsControlGains(int slot);

    /**
     * Sets the subsystem to the manually set velocity through Elastic. This resets with every code reload.
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @see #updateHopperManualVelocity(AngularVelocity)
     * @return A {@link Runnable} setting the subsystem manual target velocity
     */
    Runnable enableHopperManualVelocity();

    /**
     * Only if applicable.
     * <p>Sets the subsystem to the preset velocity stored in constants.</p>
     * <p>This does not change live, only in-code.</p>
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @return A {@link Runnable} setting the subsystem preset target velocity
     */
    Runnable enableHopperPresetVelocity();

    /**
     * Disables the subsystem motors.
     * @return A {@link Runnable} stopping the subsystem
     */
    Runnable stopHopper();

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     * @return A {@link Runnable} coasting all subsystem motors
     */
    Runnable coastHopperMotors();

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     * @return A {@link Runnable} braking all subsystem motors
     */
    Runnable brakeHopperMotors();

    /**
     * Applies the configuration inside {@link HopperConstants.PhoenixMotorConfiguration}. Follower commands are included.
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
    class DummyHopperIO implements HopperIO {

        @Override
        public void updateHopperInputs(HopperIOInputs inputs,
                                       MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {

        }

        @Override
        public void updateHopperManualVelocity(AngularVelocity newHopperManualVelocity) {

        }

        @Override
        public void updateHopperMotorsControlGains(int slot) {

        }

        @Override
        public Runnable enableHopperManualVelocity() {
            return null;
        }

        @Override
        public Runnable enableHopperPresetVelocity() {
            return null;
        }

        @Override
        public Runnable stopHopper() {
            return null;
        }

        @Override
        public Runnable coastHopperMotors() {
            return null;
        }

        @Override
        public Runnable brakeHopperMotors() {
            return null;
        }

        @Override
        public void initialMotorConfiguration() {

        }
    }
}
