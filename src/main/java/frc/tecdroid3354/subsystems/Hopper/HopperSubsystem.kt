package frc.tecdroid3354.subsystems.Hopper

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsTunableTargets
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.rotationsPerMinute
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

class HopperSubsystem(private val io: HopperIO) : SubsystemBase(HopperConstants.Telemetry.SUBSYSTEM_TAB) {
    // Auto generated file (by @AutoLog annotation in IO Layer)
    private val inputs: HopperIOInputsAutoLogged = HopperIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    /**
     * START OF CONNECTION ALERT VARIABLES. These alerts are published separately from other inputs.
     * This is to make sure all connection alerts are found in a shared folder.
     */
    private val leadMotorConnectionAlert: Alert =
        Alert(HopperConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    private val followerMotorConnectionAlert: Alert =
        Alert(HopperConstants.Telemetry.FOLLOWER_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
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
     * Updates and logs all inputs defined in [HopperIO.HopperIOInputs].
     * Updates connection alerts based on inputs.
     * Listens to updates through [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]
     */
    override fun periodic() {
        // IMPORTANT: This must be the first line in periodic() so that all other methods work with fresh data.
        io.updateHopperInputs(inputs, leadMotorInputs, followerMotorInputs)

        // Logs every field to the specified directory. It can be seen live through Elastic & AdvantageScope.
        Logger.processInputs(HopperConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(HopperConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(HopperConstants.Telemetry.FOLLOWER_MOTOR_INPUTS_TAB, followerMotorInputs)

        // Update motor alerts based on inputs.
        leadMotorConnectionAlert.set(leadMotorInputs.isConnected.not())
        followerMotorConnectionAlert.set(followerMotorInputs.isConnected.not())

        // Check if ControlGains coefficients were changed live and update the motors.
        if (SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateHopperMotorsControlGains(0) // Updates Slot0 because is the primary set
        }

        // Check if the manual target RPMs were changed live and update the target.
        if (SubsystemsTunableTargets.HOPPER_MANUAL_RPM.hasChanged(hashCode())) {
            io.updateHopperManualVelocity(
                SubsystemsTunableTargets.HOPPER_MANUAL_RPM.get().rotationsPerMinute)
        }
    }

    /**
     * Enables live-tuned velocity. See implementation comment for details.
     */
    fun enableHopperManualVelocity(): Runnable {
        return io.enableHopperManualVelocity()
    }

    /**
     * Enables pre-stored velocity. See implementation comment for details.
     */
    fun enableHopperPresetVelocity(): Runnable {
        return io.enableHopperPresetVelocity()
    }

    /**
     * Stops the subsystem. See implementation for details.
     */
    fun stopHopper(): Runnable {
        return io.stopHopper()
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Coast.
     */
    fun coastHopperMotors(): Command {
        return io.coastHopperMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Brake.
     */
    fun brakeHopperMotors(): Command {
        return io.brakeHopperMotors().InstantCommandIgnoreDisabled(this)
    }
}