package frc.tecdroid3354.subsystems.Flywheel

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.MutAngularVelocity
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.rotationsPerMinute

/**
 * Hardware layer for TalonFX motor controllers. Only file where [com.ctre.phoenix6.hardware.TalonFX]
 * motors are instantiated for this subsystem.
 *
 * **NOTE:** All methods implemented from the interface will inherit their comments. It is not necessary to repeat
 * those comments here.
 */
class FlywheelIOTalonFX: FlywheelIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.LEAD_MOTOR_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)
    private val followerMotorLeftController: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.FOLLOWER_LEFT_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)
    private val followerMotorRightOneController: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.FOLLOWER_RIGHT_ONE_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)
    private val followerMotorRightTwoController: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.FOLLOWER_RIGHT_TWO_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)

    /**
     * Note that [flywheelVelocityTarget] may contain the same value as [manualFlywheelVelocityTarget] when
     * [enableFlywheelManualVelocity] is commanded.
     */
    private val manualFlywheelVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val flywheelVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateFlywheelInputs(inputs: FlywheelIO.FlywheelIOInputs,
                                      leadMotorInputs: MotorIO.MotorIOInputs, followerMotorLeftInputs: MotorIO.MotorIOInputs,
                                      followerMotorRightOneInputs: MotorIO.MotorIOInputs, followerMotorRightTwoInputs: MotorIO.MotorIOInputs) {
        inputs.flywheelActualVelocity.mut_replace(leadMotorController.getMotorToAngularSubsystemVelocity(
            FlywheelConstants.Mechanical.REDUCTION
        ))
        inputs.flywheelTargetVelocity.mut_replace(flywheelVelocityTarget)
        inputs.flywheelManualTargetVelocity.mut_replace(manualFlywheelVelocityTarget)
        inputs.flywheelPresetVelocity.mut_replace(SubsystemsPresetTargets.FLYWHEEL_PRESET_RPM)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorLeftController.updateInputs(followerMotorLeftInputs)
        followerMotorRightOneController.updateInputs(followerMotorRightOneInputs)
        followerMotorRightTwoController.updateInputs(followerMotorRightTwoInputs)
    }

    override fun updateFlywheelManualVelocity(newFlywheelManualVelocity: AngularVelocity) {
        manualFlywheelVelocityTarget.mut_replace(newFlywheelManualVelocity)
    }

    override fun updateFlywheelMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorLeftController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorRightOneController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorRightTwoController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableFlywheelManualVelocity(): Runnable {
        return {
            flywheelVelocityTarget.mut_replace(manualFlywheelVelocityTarget) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.FLYWHEEL_CONTROL_TYPE,
                manualFlywheelVelocityTarget,
                SubsystemsMovementLimits.FLYWHEEL_VELOCITY_LIMITS,
                FlywheelConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun enableFlywheelPresetVelocity(): Runnable {
        return {
            flywheelVelocityTarget.mut_replace(SubsystemsPresetTargets.FLYWHEEL_PRESET_RPM) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.FLYWHEEL_CONTROL_TYPE,
                SubsystemsPresetTargets.FLYWHEEL_PRESET_RPM,
                SubsystemsMovementLimits.FLYWHEEL_VELOCITY_LIMITS,
                FlywheelConstants.Mechanical.REDUCTION
            )
        }
    }
    override fun enableFlywheelCalculatedVelocity(flywheelCalculatedVelocity: AngularVelocity): Runnable {
        return {
            flywheelVelocityTarget.mut_replace(flywheelCalculatedVelocity) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.FLYWHEEL_CONTROL_TYPE,
                flywheelCalculatedVelocity,
                SubsystemsMovementLimits.FLYWHEEL_VELOCITY_LIMITS,
                FlywheelConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopFlywheel(): Runnable {
        return {
            flywheelVelocityTarget.mut_replace(0.0.rotationsPerMinute)
            leadMotorController.stopMotor()
        }
    }

    override fun coastFlywheelMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorLeftController.coast()
            followerMotorRightOneController.coast()
            followerMotorRightTwoController.coast()
        }
    }

    override fun brakeFlywheelMotors(): Runnable {
        return {
            leadMotorController.brake()
            followerMotorLeftController.brake()
            followerMotorRightOneController.brake()
            followerMotorRightTwoController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorLeftController.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorRightOneController.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorRightTwoController.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorLeftController.follow(
            leadMotorController.getMotorInstance(),
            FlywheelConstants.PhoenixMotorConfiguration.followerLeftMotorAlignment)

        followerMotorRightOneController.follow(
            leadMotorController.getMotorInstance(),
            FlywheelConstants.PhoenixMotorConfiguration.followerRightMotorAlignment)
        followerMotorRightTwoController.follow(
            leadMotorController.getMotorInstance(),
            FlywheelConstants.PhoenixMotorConfiguration.followerRightMotorAlignment)
    }
}