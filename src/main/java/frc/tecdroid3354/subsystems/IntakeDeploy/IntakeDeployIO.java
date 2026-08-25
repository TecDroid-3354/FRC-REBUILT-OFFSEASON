package frc.tecdroid3354.subsystems.IntakeDeploy;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link Distance} driven subsystem, i.e. an Elevator.
 * Duplicate the motor-related fields inside {@link IntakeDeployIOInputs} for the number of motors in your subsystem,
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
 *  - When using kapt to generate classes, we have to mark every field of the {@link IntakeDeployIOInputs}
 *      as @JvmField, which is just annoying and impacts readability.
 *  </p>
 * </p>
 */
public interface IntakeDeployIO {
    /**
     * Class intended to log all relevant fields that might change during a match.
     * These inputs may be used for:
     *  <p>- Creating alerts (i.e. when a motor disconnects)</p>
     *  <p>- Tune ControlGains constants in AdvantageScope (using Mechanical Advantage's LoggedTunableNumber)</p>
     *  <p>- Replay a match in AdvantageScope (using Replay Mode)</p>
     *
     *  All of these fields should be published periodically through
     *  {@link org.littletonrobotics.junction.Logger Logger.processInputs(String, IOInputsAutoLogged)}
     *
     *  <p>
     *      If you are having trouble with your generated {@link IntakeDeployIOInputsAutoLogged}, it is most probably
     *      because of wrong kapt configuration. Ask an Area Lead or Captain.
     *  </p>
     */
    @AutoLog
    class IntakeDeployIOInputs {
        /** Subsystem wise fields */
        public MutDistance intakeDeployDisplacement = Meters.mutable(0.0);          // Actual Displacement
        public MutDistance intakeDeployTargetDisplacement = Meters.mutable(0.0);    // Setpoint / Target
        public MutDistance intakeDeployManualTargetDisplacement = Meters.mutable(0.0); // Manually set live
    }

    /**
     * Intended to update any relevant fields in {@link IntakeDeployIOInputs IntakeDeployIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link IntakeDeployIOInputsAutoLogged} object keeping track of everything.
     */
    void updateIntakeDeployInputs(IntakeDeployIOInputs inputs,
                                  MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs);

    /**
     * Used to update the in-file variable containing the manual target displacement. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newIntakeDeployManualDisplacement Obtained live through Elastic.
     */
    void updateIntakeDeployManualDisplacement(Distance newIntakeDeployManualDisplacement);

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    void updateIntakeDeployMotorsControlGains(int slot);

    /**
     * Changes the target displacement of the subsystem to the one set
     * in {@link #updateIntakeDeployManualDisplacement(Distance)}
     */
    Runnable setIntakeDeployManualTargetDisplacement();

    /**
     * Changes the target displacement of the subsystem.
     * @param intakeDeployTargetDisplacement The desired displacement for the subsystem.
     */
    Runnable setIntakeDeployTargetDisplacement(Distance intakeDeployTargetDisplacement);

    /**
     * Stops the subsystem motors. Intended for SysId, or an abrupt manual stop.
     */
    Runnable stopIntakeDeploy();

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     */
    Runnable coastIntakeDeployMotors();

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     */
    Runnable brakeIntakeDeployMotors();

    /**
     * Applies the configuration inside {@link IntakeDeployConstants.PhoenixMotorConfiguration}. Follower commands are included.
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
    class DummyIntakeDeployIO implements IntakeDeployIO {

        @Override
        public void updateIntakeDeployInputs(IntakeDeployIOInputs inputs,
                                             MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {

        }

        @Override
        public void updateIntakeDeployManualDisplacement(Distance newIntakeDeployManualDisplacement) {

        }

        @Override
        public void updateIntakeDeployMotorsControlGains(int slot) {

        }

        @Override
        public Runnable setIntakeDeployManualTargetDisplacement() {
            return null;
        }

        @Override
        public Runnable setIntakeDeployTargetDisplacement(Distance intakeDeployTargetDisplacement) {
            return null;
        }

        @Override
        public Runnable stopIntakeDeploy() {
            return null;
        }

        @Override
        public Runnable coastIntakeDeployMotors() {
            return null;
        }

        @Override
        public Runnable brakeIntakeDeployMotors() {
            return null;
        }

        @Override
        public void initialMotorConfiguration() {

        }
    }
}
