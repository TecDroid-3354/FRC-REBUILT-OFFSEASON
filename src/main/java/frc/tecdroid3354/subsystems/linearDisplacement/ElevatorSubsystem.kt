package frc.tecdroid3354.subsystems.linearDisplacement

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.constants.SubsystemsTunableTargets
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import org.littletonrobotics.junction.Logger

/**
 * Intended to act as a bridge between the I/O layer and the rest of the program.
 * All logic regarding the elevator behaviour should be performed here, as the I/O must receive exclusively orders to
 * pass to either the hardware or simulation, depending on the implementation.
 */
class ElevatorSubsystem(private val io: ElevatorIO): SubsystemBase(ElevatorConstants.Telemetry.SUBSYSTEM_TAB) {
    /**
     * I/O (Input/Output) variables ([io] passed in constructor). Enables to use different implementations of
     * [ElevatorIO] without modifying this class.
     */
    private val inputs: ElevatorIOInputsAutoLogged = ElevatorIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    /**
     * Alerts to inform driver / developers something went wrong
     */
    val leadMotorDisconnectedAlert: Alert =
        Alert(ElevatorConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    val followerMotorDisconnectedAlert: Alert =
        Alert(ElevatorConstants.Telemetry.FOLLOWER_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)

    /**
     * Used for all sensors / actuators configuration.
     */
    init {
        io.initialMotorConfiguration()
    }

    /**
     * Called every 20ms. Updates every input according to the I/O implementation and logs it.
     */
    override fun periodic() {
        io.updateElevatorInputs(inputs, leadMotorInputs, followerMotorInputs)
        // Make sure is AdvantageKit's Logger (org.littletonrobotics.junction) and not Java's.
        Logger.processInputs(ElevatorConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(ElevatorConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(ElevatorConstants.Telemetry.FOLLOWER_MOTOR_INPUTS_TAB, followerMotorInputs)

        // Updates each alert based on the retrieved connectivity status of this cycle.
        // alert = notConnected ? true : false
        leadMotorDisconnectedAlert.set(leadMotorInputs.isConnected.not())
        followerMotorDisconnectedAlert.set(followerMotorInputs.isConnected.not())

        if (SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateElevatorMotorsControlGains(0) // Updates slot0 because is the primary set
        }

        if (SubsystemsTunableTargets.ELEVATOR_MANUAL_TARGET_INCHES.hasChanged(hashCode())) {
            io.updateElevatorManualDisplacement(SubsystemsTunableTargets.ELEVATOR_MANUAL_TARGET_INCHES.get().inches)
        }
    }

    fun setElevatorManualTargetDisplacement(): Runnable {
        return io.setElevatorManualTargetDisplacement()
    }
    /**
     * Clamps the desired [targetDisplacement] within the limits defined in [ElevatorConstants] and
     * passes the result to the [io] layer to command the motors.
     * @param targetDisplacement The desired target displacement of the [ElevatorSubsystem] (NOT the motors).
     */
    fun setElevatorTargetDisplacement(targetDisplacement: Distance): Runnable {
        return io.setElevatorTargetDisplacement(targetDisplacement);
    }

    fun setElevatorIdleDisplacement(): Runnable {
        return io.setElevatorTargetDisplacement(SubsystemsPresetTargets.ELEVATOR_IDLE_DISPLACEMENT)
    }

    fun setElevatorHomeDisplacement(): Runnable {
        return io.setElevatorTargetDisplacement(SubsystemsPresetTargets.ELEVATOR_HOME_DISPLACEMENT)
    }

    fun getElevatorDisplacement(): Distance = inputs.elevatorDisplacement

    /**
     * Fabricates an [InstantCommand] switching the Neutral / Idle mode of the motors to coast through the I/O layer.
     * @return an [InstantCommand] that coasts the [ElevatorSubsystem] motors.
     */
    fun coastElevatorMotors(): Command {
        return io.coastElevatorMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Fabricates an [InstantCommand] switching the Neutral / Idle mode of the motors to brake through the I/O layer.
     * @return an [InstantCommand] that brakes the [ElevatorSubsystem] motors.
     */
    fun brakeElevatorMotors(): Command {
        return io.brakeElevatorMotors().InstantCommandIgnoreDisabled(this)
    }
}
