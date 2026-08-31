package frc.tecdroid3354.subsystems.Hood;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link Angle} driven subsystem, i.e. a Joint.
 * Duplicate the motor-related fields inside {@link HoodIOInputs} for the number of motors in your subsystem,
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
 *  - When using kapt to generate classes, we have to mark every field of the {@link HoodIOInputs}
 *      as @JvmField, which is just annoying and impacts readability.
 *  </p>
 * </p>
 */
public interface HoodIO {
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
     *      If you are having trouble with your generated {@link HoodIOInputsAutoLogged}, it is most probably
     *      because of wrong kapt configuration. Ask an Area Lead or Captain.
     *  </p>
     */
    @AutoLog
    class HoodIOInputs {
        /** Subsystem wise fields */
        public MutAngle hoodActualPosition = Degrees.mutable(0.0);
        public MutAngle hoodTargetPosition = Degrees.mutable(0.0);
        public MutAngle hoodManualTargetPosition = Degrees.mutable(0.0);
    }

    /**
     * Intended to update any relevant fields in {@link HoodIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link HoodIOInputs} object keeping track of everything.
     */
    void updateHoodInputs(HoodIOInputs inputs, MotorIO.MotorIOInputs leadMotorInputs);

    /**
     * Used to update the in-file variable containing the manual target position. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newHoodPosition Obtained live through Elastic.
     */
    void updateHoodManualPosition(Angle newHoodPosition);

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    void updateHoodMotorsControlGains(int slot);

    /**
     * Sets the subsystem to the manually set position through Elastic. This resets with every code reload.
     * <p>Make sure to update your subsystem target position variable for telemetry</p>
     * @see #updateHoodManualPosition(Angle)
     * @return A {@link Runnable} setting the subsystem manual target position
     */
    Runnable setHoodManualPosition();

    /**
     * Sets the subsystem to the given position.
     * <p>Make sure to update your subsystem target position variable telemetry</p>
     * @param HoodPosition The desired position in subsystem units
     * @return A {@link Runnable} setting the subsystem to the target position
     */
    Runnable setHoodPosition(Angle HoodPosition);

    /**
     * Disables the subsystem motors.
     * @return A {@link Runnable} stopping the subsystem
     */
    Runnable stopHood();

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     * @return A {@link Runnable} coasting all subsystem motors
     */
    Runnable coastHoodMotors();

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     * @return A {@link Runnable} braking all subsystem motors
     */
    Runnable brakeHoodMotors();

    /**
     * Applies the configuration inside {@link frc.tecdroid3354.subsystems.Hood.HoodConstants.PhoenixMotorConfiguration}. Follower commands are included.
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
    class DummyHoodIO implements HoodIO {

        @Override
        public void updateHoodInputs(HoodIOInputs inputs, MotorIO.MotorIOInputs leadMotorInputs) {

        }

        @Override
        public void updateHoodManualPosition(Angle newHoodPosition) {

        }

        @Override
        public void updateHoodMotorsControlGains(int slot) {

        }

        @Override
        public Runnable setHoodManualPosition() {
            return null;
        }

        @Override
        public Runnable setHoodPosition(Angle HoodPosition) {
            return null;
        }

        @Override
        public Runnable stopHood() {
            return null;
        }

        @Override
        public Runnable coastHoodMotors() {
            return null;
        }

        @Override
        public Runnable brakeHoodMotors() {
            return null;
        }

        @Override
        public void initialMotorConfiguration() {

        }
    }
}

