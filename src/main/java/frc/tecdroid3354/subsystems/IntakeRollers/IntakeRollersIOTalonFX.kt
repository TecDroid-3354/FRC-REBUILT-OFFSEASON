package frc.tecdroid3354.subsystems.IntakeRollers

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.MutAngularVelocity
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.interfaces.MotorIO

/**
 * Hardware layer for TalonFX motor controllers. Only file where [com.ctre.phoenix6.hardware.TalonFX]
 * motors are instantiated for this subsystem.
 *
 * **NOTE:** All methods implemented from the interface will inherit their comments. It is not necessary to repeat
 * those comments here.
 */
class IntakeRollersIOTalonFX: IntakeRollersIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        IntakeRollersConstants.Identification.LEAD_MOTOR_ID,
        IntakeRollersConstants.Identification.INTAKE_ROLLERS_CANBUS_NAME)
    private val followerMotorController: OpTalonFX = OpTalonFX(
        IntakeRollersConstants.Identification.FOLLOWER_MOTOR_ID,
        IntakeRollersConstants.Identification.INTAKE_ROLLERS_CANBUS_NAME)

    /**
     * Note that [intakeRollersVelocityTarget] may contain the same value as [manualIntakeRollersVelocityTarget] when
     * [enableIntakeRollersManualVelocity] is commanded.
     */
    private val manualIntakeRollersVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val intakeRollersVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateIntakeRollersInputs(inputs: IntakeRollersIO.IntakeRollersIOInputs,
                                           leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        inputs.intakeRollersActualVelocity.mut_replace(leadMotorController.getMotorToAngularSubsystemVelocity(
            IntakeRollersConstants.Mechanical.REDUCTION
        ))
        inputs.intakeRollersTargetVelocity.mut_replace(intakeRollersVelocityTarget)
        inputs.intakeRollersManualTargetVelocity.mut_replace(manualIntakeRollersVelocityTarget)
        inputs.intakeRollersPresetVelocity.mut_replace(SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorController.updateInputs(followerMotorInputs)
    }

    override fun updateIntakeRollersManualVelocity(newIntakeRollersManualVelocity: AngularVelocity) {
        manualIntakeRollersVelocityTarget.mut_replace(newIntakeRollersManualVelocity)
    }

    override fun updateIntakeRollersMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = IntakeRollersConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> {
                newMotorsConfig.Slot1 = SubsystemsControlGains.INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableIntakeRollersManualVelocity(): Runnable {
        return {
            intakeRollersVelocityTarget.mut_replace(manualIntakeRollersVelocityTarget) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.INTAKE_ROLLERS_CONTROL_TYPE,
                manualIntakeRollersVelocityTarget,
                SubsystemsMovementLimits.INTAKE_ROLLERS_VELOCITY_LIMITS,
                IntakeRollersConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun enableIntakeRollersPresetVelocity(): Runnable {
        return {
            intakeRollersVelocityTarget.mut_replace(SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.INTAKE_ROLLERS_CONTROL_TYPE,
                SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM,
                SubsystemsMovementLimits.INTAKE_ROLLERS_VELOCITY_LIMITS,
                IntakeRollersConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopIntakeRollers(): Runnable {
        return {
            leadMotorController.stopMotor()
        }
    }

    override fun coastIntakeRollersMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorController.coast()
        }
    }

    override fun brakeIntakeRollersMotors(): Runnable {
        return {
            leadMotorController.brake()
            followerMotorController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(IntakeRollersConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorController.applyConfigAndClearFaults(IntakeRollersConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorController.follow(
            leadMotorController.getMotorInstance(),
            IntakeRollersConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}