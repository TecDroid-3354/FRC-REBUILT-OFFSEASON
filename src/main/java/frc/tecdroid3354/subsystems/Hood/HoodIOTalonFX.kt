package frc.tecdroid3354.subsystems.Hood

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX

class HoodIOTalonFX: HoodIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        HoodConstants.Identification.LEAD_MOTOR_ID, HoodConstants.Identification.HOOD_CANBUS_NAME
    )

    private val hoodTargetPosition: MutAngle = Degrees.mutable(0.0)
    private val hoodManualTargetPosition: MutAngle = Degrees.mutable(0.0)

    override fun updateHoodInputs(inputs: HoodIO.HoodIOInputs, leadMotorInputs: MotorIO.MotorIOInputs) {
        inputs.hoodActualPosition.mut_replace(leadMotorController.getMotorToAngularSubsystemPosition(
            HoodConstants.Mechanical.REDUCTION
        ))
        inputs.hoodTargetPosition.mut_replace(hoodTargetPosition)
        inputs.hoodManualTargetPosition.mut_replace(hoodManualTargetPosition)

        leadMotorController.updateInputs(leadMotorInputs)
    }

    override fun updateHoodManualPosition(newHoodPosition: Angle) {
        hoodManualTargetPosition.mut_replace(newHoodPosition)
    }

    override fun updateHoodMotorsControlGains(slot: Int) {
        val validatedSlot = MathUtil.clamp(slot, 0, 2) // Make sure the selected slot is either 0, 1, or 2
        // Clone the initial config
        val newMotorsConfig = HoodConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> { // Secondary not declared
                newMotorsConfig.Slot1 = SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun setHoodManualPosition(): Runnable {
        return {
            hoodTargetPosition.mut_replace(hoodManualTargetPosition)

            leadMotorController.angularSubsystemPositionRequest(
                SubsystemsControlRequests.HOOD_CONTROL_TYPE,
                hoodManualTargetPosition,
                SubsystemsMovementLimits.HOOD_POSITION_LIMITS,
                HoodConstants.Mechanical.REDUCTION,
            )
        }
    }

    override fun setHoodPosition(hoodPosition: Angle): Runnable {
        return {
            hoodTargetPosition.mut_replace(hoodPosition)

            leadMotorController.angularSubsystemPositionRequest(
                SubsystemsControlRequests.HOOD_CONTROL_TYPE,
                hoodPosition,
                SubsystemsMovementLimits.HOOD_POSITION_LIMITS,
                HoodConstants.Mechanical.REDUCTION,
            )
        }
    }

    override fun stopHood(): Runnable {
        return { leadMotorController.stopMotor() }
    }

    override fun coastHoodMotors(): Runnable {
        return {
            leadMotorController.coast()
        }
    }

    override fun brakeHoodMotors(): Runnable {
        return {
            leadMotorController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(HoodConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
    }
}