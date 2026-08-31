package frc.tecdroid3354.subsystems.IntakeDeploy

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutDistance
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployConstants.Identification
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployConstants.Mechanical
import frc.tecdroid3354.utils.interfaces.MotorIO
import java.util.Optional

/**
 * Hardware layer for TalonFX motor controllers. Only file where [com.ctre.phoenix6.hardware.TalonFX]
 * motors are instantiated for this subsystem.
 *
 * **NOTE:** All methods implemented from the interface will inherit their comments. It is not necessary to repeat
 * those comments here.
 */
class IntakeDeployIOTalonFX : IntakeDeployIO {
    // Make sure to configure it.
    private val leadMotorController : OpTalonFX = OpTalonFX(Identification.LEAD_MOTOR_ID,
                                                            Identification.INTAKE_DEPLOY_CANBUS_NAME)

    private val intakeDeployTargetDisplacement      : MutDistance = Meters.mutable(0.0)
    private val intakeDeployManualTargetDisplacement: MutDistance = Meters.mutable(0.0)

    override fun updateIntakeDeployInputs(inputs: IntakeDeployIO.IntakeDeployIOInputs, leadMotorInputs: MotorIO.MotorIOInputs) {
        inputs.intakeDeployDisplacement.mut_replace(leadMotorController.getMotorToLinearSubsystemDisplacement(
            Mechanical.REDUCTION, Mechanical.SPROCKET
        ))
        inputs.intakeDeployTargetDisplacement.mut_replace(intakeDeployTargetDisplacement)
        inputs.intakeDeployManualTargetDisplacement.mut_replace(intakeDeployManualTargetDisplacement)

        leadMotorController.updateInputs(leadMotorInputs)
    }

    override fun updateIntakeDeployManualDisplacement(newIntakeDeployManualDisplacement: Distance) {
        this.intakeDeployManualTargetDisplacement.mut_replace(newIntakeDeployManualDisplacement)
    }

    override fun updateIntakeDeployMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = IntakeDeployConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> { // Note that only primary gains are declared, hence is the only one in use for all slots
                newMotorsConfig.Slot0 = SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> {
                newMotorsConfig.Slot1 = SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, since we're using the validatedSlot
                newMotorsConfig.Slot2 = SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun setIntakeDeployManualTargetDisplacement(): Runnable {
        return {
            this.intakeDeployTargetDisplacement.mut_replace(intakeDeployManualTargetDisplacement)

            leadMotorController.linearSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.INTAKE_DEPLOY_CONTROL_TYPE,
                intakeDeployManualTargetDisplacement,
                SubsystemsMovementLimits.INTAKE_DEPLOY_DISPLACEMENT_LIMITS,
                Mechanical.SPROCKET,
                Mechanical.REDUCTION,
                // Note how the SECONDARY motion targets are used when using manual target displacement
                Optional.of(SubsystemsMotionTargets.INTAKE_DEPLOY_SECONDARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun setIntakeDeployTargetDisplacement(intakeDeployTargetDisplacement: Distance): Runnable {
        return {
            this.intakeDeployTargetDisplacement.mut_replace(intakeDeployTargetDisplacement)

            leadMotorController.linearSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.INTAKE_DEPLOY_CONTROL_TYPE,
                intakeDeployTargetDisplacement,
                SubsystemsMovementLimits.INTAKE_DEPLOY_DISPLACEMENT_LIMITS,
                Mechanical.SPROCKET,
                Mechanical.REDUCTION,
                Optional.of( // For Clustered position, secondary motion targets will be used
                    if (this.intakeDeployTargetDisplacement == SubsystemsPresetTargets.INTAKE_DEPLOY_CLUSTERING_DISPLACEMENT)
                        SubsystemsMotionTargets.INTAKE_DEPLOY_SECONDARY_MOTION_TARGETS
                    else SubsystemsMotionTargets.INTAKE_DEPLOY_PRIMARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun stopIntakeDeploy(): Runnable {
        return { leadMotorController.stopMotor() }
    }

    override fun coastIntakeDeployMotors(): Runnable {
        return {
            leadMotorController.coast()
        }
    }

    override fun brakeIntakeDeployMotors(): Runnable {
       return {
            leadMotorController.brake()
       }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(IntakeDeployConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
    }
}