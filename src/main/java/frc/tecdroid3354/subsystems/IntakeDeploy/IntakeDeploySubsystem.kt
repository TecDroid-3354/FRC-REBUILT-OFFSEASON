package frc.tecdroid3354.subsystems.IntakeDeploy

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemTolerances
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.constants.SubsystemsTunableTargets
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

/**
 * Intended to act as a bridge between the I/O layer and the rest of the program.
 * All logic regarding the subsystem behaviour should be performed here, as the I/O must receive exclusively orders to
 * pass to either the hardware or simulation, depending on the implementation.
 */
class IntakeDeploySubsystem(private val io: IntakeDeployIO): SubsystemBase(IntakeDeployConstants.Telemetry.SUBSYSTEM_TAB) {
    /**
     * I/O (Input/Output) variables ([io] passed in constructor). Enables to use different implementations of
     * [IntakeDeployIO] without modifying this class.
     */
    private val inputs: IntakeDeployIOInputsAutoLogged = IntakeDeployIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    /**
     * Alerts to inform driver / developers something went wrong
     */
    val leadMotorDisconnectedAlert: Alert =
        Alert(IntakeDeployConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)

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
        io.updateIntakeDeployInputs(inputs, leadMotorInputs)
        // Make sure is AdvantageKit's Logger (org.littletonrobotics.junction) and not Java's.
        Logger.processInputs(IntakeDeployConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(IntakeDeployConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)

        // Updates each alert based on the retrieved connectivity status of this cycle.
        // alert = notConnected ? true : false
        leadMotorDisconnectedAlert.set(leadMotorInputs.isConnected.not())

        if (SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateIntakeDeployMotorsControlGains(0) // Updates slot0 because is the primary set
        }

        if (SubsystemsTunableTargets.INTAKE_DEPLOY_MANUAL_TARGET_INCHES.hasChanged(hashCode())) {
            io.updateIntakeDeployManualDisplacement(SubsystemsTunableTargets.INTAKE_DEPLOY_MANUAL_TARGET_INCHES.get().inches)
        }
    }

    fun setIntakeDeployManualTargetDisplacement(): Runnable {
        return io.setIntakeDeployManualTargetDisplacement()
    }
    /**
     * Clamps the desired [targetDisplacement] within the limits defined in [IntakeDeployConstants] and
     * passes the result to the [io] layer to command the motors.
     * @param targetDisplacement The desired target displacement of the [IntakeDeploySubsystem] (NOT the motors).
     */
    fun setIntakeDeployTargetDisplacement(targetDisplacement: Distance): Runnable {
        return io.setIntakeDeployTargetDisplacement(targetDisplacement);
    }

    fun setIntakeDeployExtendedDisplacement(): Runnable {
        return io.setIntakeDeployTargetDisplacement(SubsystemsPresetTargets.INTAKE_DEPLOY_EXTENDED_DISPLACEMENT)
    }

    fun setIntakeDeployClusteringDisplacement(): Runnable {
        return io.setIntakeDeployTargetDisplacement(SubsystemsPresetTargets.INTAKE_DEPLOY_CLUSTERING_DISPLACEMENT)
    }

    fun setIntakeDeployHomeDisplacement(): Runnable {
        return io.setIntakeDeployTargetDisplacement(SubsystemsPresetTargets.INTAKE_DEPLOY_HOME_DISPLACEMENT)
    }

    fun getIntakeDeployDisplacement(): Distance = inputs.intakeDeployDisplacement

    fun getIsDeployAtTarget(): Boolean = abs(inputs.intakeDeployTargetDisplacement
        .minus(inputs.intakeDeployDisplacement).inches) < SubsystemTolerances.INTAKE_DEPLOY_TOLERANCE.inches

    fun getIsDeployed(): Boolean = abs(inputs.intakeDeployDisplacement
        .minus(SubsystemsPresetTargets.INTAKE_DEPLOY_EXTENDED_DISPLACEMENT).inches) < SubsystemTolerances.INTAKE_DEPLOY_TOLERANCE.inches

    /**
     * Fabricates an [InstantCommand] switching the Neutral / Idle mode of the motors to coast through the I/O layer.
     * @return an [InstantCommand] that coasts the [IntakeDeploySubsystem] motors.
     */
    fun coastIntakeDeployMotors(): Command {
        return io.coastIntakeDeployMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Fabricates an [InstantCommand] switching the Neutral / Idle mode of the motors to brake through the I/O layer.
     * @return an [InstantCommand] that brakes the [IntakeDeploySubsystem] motors.
     */
    fun brakeIntakeDeployMotors(): Command {
        return io.brakeIntakeDeployMotors().InstantCommandIgnoreDisabled(this)
    }
}
