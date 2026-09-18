package frc.tecdroid3354.subsystems.IntakeRollers

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.constants.SubsystemsTunableTargets
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import frc.tecdroid3354.utils.rotationsPerMinute
import org.littletonrobotics.junction.Logger

class IntakeRollersSubsystem(private val io: IntakeRollersIO) : SubsystemBase(IntakeRollersConstants.Telemetry.SUBSYSTEM_TAB) {
    // Auto generated file (by @AutoLog annotation in IO Layer)
    private val inputs: IntakeRollersIOInputsAutoLogged = IntakeRollersIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    /**
     * START OF CONNECTION ALERT VARIABLES. These alerts are published separately from other inputs.
     * This is to make sure all connection alerts are found in a shared folder.
     */
    private val leadMotorConnectionAlert: Alert =
        Alert(IntakeRollersConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    private val followerMotorConnectionAlert: Alert =
        Alert(IntakeRollersConstants.Telemetry.FOLLOWER_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    /**
     * END OF CONNECTION ALERT VARIABLES
     */

    /**
     * Used for all sensors / actuators configuration.
     */
    init {
        io.initialMotorConfiguration()
    }

    /**
     * Updates and logs all inputs defined in [IntakeRollersIO.IntakeRollersIOInputs].
     * Updates connection alerts based on inputs.
     * Listens to updates through [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]
     */
    override fun periodic() {
        // IMPORTANT: This must be the first line in periodic() so that all other methods work with fresh data.
        io.updateIntakeRollersInputs(inputs, leadMotorInputs, followerMotorInputs)

        // Logs every field to the specified directory. It can be seen live through Elastic & AdvantageScope.
        Logger.processInputs(IntakeRollersConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(IntakeRollersConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(IntakeRollersConstants.Telemetry.FOLLOWER_MOTOR_INPUTS_TAB, followerMotorInputs)

        // Update motor alerts based on inputs.
        leadMotorConnectionAlert.set(leadMotorInputs.isConnected.not())
        followerMotorConnectionAlert.set(followerMotorInputs.isConnected.not())

        // Check if ControlGains coefficients were changed live and update the motors.
        if (SubsystemsControlGains.INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateIntakeRollersMotorsControlGains(0) // Updates Slot0 because is the primary set
        }

        // Check if the manual target RPMs were changed live and update the target.
        if (SubsystemsTunableTargets.INTAKE_ROLLERS_MANUAL_RPM.hasChanged(hashCode())) {
            io.updateIntakeRollersManualVelocity(
                SubsystemsTunableTargets.INTAKE_ROLLERS_MANUAL_RPM.get().rotationsPerMinute)
        }
    }

    /**
     * Enables live-tuned velocity. See implementation comment for details.
     */
    fun enableIntakeRollersManualVelocity(): Runnable {
        return io.enableIntakeRollersManualVelocity()
    }

    /**
     * Enables pre-stored velocity. See implementation comment for details.
     */
    fun enableIntakeRollersPresetVelocity(): Runnable {
        return io.enableIntakeRollersVelocity(SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM)
    }

    fun enableOuttakeRollersPresetVelocity(): Runnable {
        return io.enableIntakeRollersVelocity(SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM.unaryMinus())
    }

    /**
     * Stops the subsystem. See implementation for details.
     */
    fun stopIntakeRollers(): Runnable {
        return io.stopIntakeRollers()
    }

    /** Checks if the rollers velocity is greater than 100 RPMs. The threshold is an arbitrary value. */
    fun getIsActive(): Boolean {
        return inputs.intakeRollersActualVelocity.gt(100.0.rotationsPerMinute)
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Coast.
     */
    fun coastIntakeRollersMotors(): Command {
        return io.coastIntakeRollersMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Brake.
     */
    fun brakeIntakeRollersMotors(): Command {
        return io.brakeIntakeRollersMotors().InstantCommandIgnoreDisabled(this)
    }

}

