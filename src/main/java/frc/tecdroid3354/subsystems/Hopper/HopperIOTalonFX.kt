package frc.tecdroid3354.subsystems.Hopper

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
class HopperIOTalonFX: HopperIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        HopperConstants.Identification.LEAD_MOTOR_ID,
        HopperConstants.Identification.HOPPER_CANBUS_NAME)
    private val followerMotorController: OpTalonFX = OpTalonFX(
        HopperConstants.Identification.FOLLOWER_MOTOR_ID,
        HopperConstants.Identification.HOPPER_CANBUS_NAME)

    /**
     * Note that [hopperVelocityTarget] may contain the same value as [manualHopperVelocityTarget] when
     * [enableHopperManualVelocity] is commanded.
     */
    private val manualHopperVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val hopperVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateHopperInputs(inputs: HopperIO.HopperIOInputs,
                                    leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        inputs.hopperActualVelocity.mut_replace(leadMotorController.getMotorToAngularSubsystemVelocity(
            HopperConstants.Mechanical.REDUCTION
        ))
        inputs.hopperTargetVelocity.mut_replace(hopperVelocityTarget)
        inputs.hopperManualTargetVelocity.mut_replace(manualHopperVelocityTarget)
        inputs.hopperPresetVelocity.mut_replace(SubsystemsPresetTargets.HOPPER_PRESET_RPM)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorController.updateInputs(followerMotorInputs)
    }

    override fun updateHopperManualVelocity(newHopperManualVelocity: AngularVelocity) {
        manualHopperVelocityTarget.mut_replace(newHopperManualVelocity)
    }

    override fun updateHopperMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = HopperConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableHopperManualVelocity(): Runnable {
        return {
            hopperVelocityTarget.mut_replace(manualHopperVelocityTarget) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.HOPPER_CONTROL_TYPE,
                manualHopperVelocityTarget,
                SubsystemsMovementLimits.HOPPER_VELOCITY_LIMITS,
                HopperConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun enableHopperPresetVelocity(): Runnable {
        return {
            hopperVelocityTarget.mut_replace(SubsystemsPresetTargets.HOPPER_PRESET_RPM) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.HOPPER_CONTROL_TYPE,
                SubsystemsPresetTargets.HOPPER_PRESET_RPM,
                SubsystemsMovementLimits.HOPPER_VELOCITY_LIMITS,
                HopperConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopHopper(): Runnable {
        return {
            leadMotorController.stopMotor()
        }
    }

    override fun coastHopperMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorController.coast()
        }
    }

    override fun brakeHopperMotors(): Runnable {
        return {
            leadMotorController.brake()
            followerMotorController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(HopperConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorController.applyConfigAndClearFaults(HopperConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorController.follow(
            leadMotorController.getMotorInstance(),
            HopperConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}