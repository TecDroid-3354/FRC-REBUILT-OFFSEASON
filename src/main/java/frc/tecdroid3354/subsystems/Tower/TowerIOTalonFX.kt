package frc.tecdroid3354.subsystems.Tower

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
import frc.tecdroid3354.utils.rotationsPerMinute

/**
 * Hardware layer for TalonFX motor controllers. Only file where [com.ctre.phoenix6.hardware.TalonFX]
 * motors are instantiated for this subsystem.
 *
 * **NOTE:** All methods implemented from the interface will inherit their comments. It is not necessary to repeat
 * those comments here.
 */
class TowerIOTalonFX: TowerIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        TowerConstants.Identification.LEAD_MOTOR_ID,
        TowerConstants.Identification.TOWER_CANBUS_NAME)
    private val followerMotorController: OpTalonFX = OpTalonFX(
        TowerConstants.Identification.FOLLOWER_MOTOR_ID,
        TowerConstants.Identification.TOWER_CANBUS_NAME)

    /**
     * Note that [towerVelocityTarget] may contain the same value as [manualTowerVelocityTarget] when
     * [enableTowerManualVelocity] is commanded.
     */
    private val manualTowerVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val towerVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateTowerInputs(inputs: TowerIO.TowerIOInputs,
                                   leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        inputs.towerActualVelocity.mut_replace(leadMotorController.getMotorToAngularSubsystemVelocity(
            TowerConstants.Mechanical.REDUCTION
        ))
        inputs.towerTargetVelocity.mut_replace(towerVelocityTarget)
        inputs.towerManualTargetVelocity.mut_replace(manualTowerVelocityTarget)
        inputs.towerPresetVelocity.mut_replace(SubsystemsPresetTargets.TOWER_PRESET_RPM)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorController.updateInputs(followerMotorInputs)
    }

    override fun updateTowerManualVelocity(newTowerManualVelocity: AngularVelocity) {
        manualTowerVelocityTarget.mut_replace(newTowerManualVelocity)
    }

    override fun updateTowerMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = TowerConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.TOWER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.TOWER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableTowerManualVelocity(): Runnable {
        return {
            towerVelocityTarget.mut_replace(manualTowerVelocityTarget) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.TOWER_CONTROL_TYPE,
                manualTowerVelocityTarget,
                SubsystemsMovementLimits.TOWER_VELOCITY_LIMITS,
                TowerConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun enableTowerPresetVelocity(): Runnable {
        return {
            towerVelocityTarget.mut_replace(SubsystemsPresetTargets.TOWER_PRESET_RPM) // Update target velocity

            leadMotorController.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.TOWER_CONTROL_TYPE,
                SubsystemsPresetTargets.TOWER_PRESET_RPM,
                SubsystemsMovementLimits.TOWER_VELOCITY_LIMITS,
                TowerConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopTower(): Runnable {
        return {
            towerVelocityTarget.mut_replace(0.0.rotationsPerMinute)
            leadMotorController.stopMotor()
        }
    }

    override fun coastTowerMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorController.coast()
        }
    }

    override fun brakeTowerMotors(): Runnable {
        return {
            leadMotorController.brake()
            followerMotorController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(TowerConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorController.applyConfigAndClearFaults(TowerConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorController.follow(
            leadMotorController.getMotorInstance(),
            TowerConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}