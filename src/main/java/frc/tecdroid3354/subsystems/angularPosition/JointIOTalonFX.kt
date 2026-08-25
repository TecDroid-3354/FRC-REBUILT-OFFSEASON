package frc.tecdroid3354.subsystems.angularPosition

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import java.util.Optional

class JointIOTalonFX: JointIO {
    private val leadMotorController: OpTalonFX = OpTalonFX(
        JointConstants.Identification.LEAD_MOTOR_ID, JointConstants.Identification.JOINT_CANBUS_NAME
    )
    private val followerMotorController: OpTalonFX = OpTalonFX(
        JointConstants.Identification.FOLLOWER_MOTOR_ID, JointConstants.Identification.JOINT_CANBUS_NAME
    )

    private val jointTargetPosition: MutAngle = Degrees.mutable(0.0)
    private val jointManualTargetPosition: MutAngle = Degrees.mutable(0.0)

    override fun updateJointInputs(inputs: JointIO.JointIOInputs,
                                   leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        inputs.jointActualPosition.mut_replace(leadMotorController.getMotorToAngularSubsystemPosition(
            JointConstants.Mechanical.REDUCTION
        ))
        inputs.jointTargetPosition.mut_replace(jointTargetPosition)
        inputs.jointManualTargetPosition.mut_replace(jointManualTargetPosition)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorController.updateInputs(followerMotorInputs)
    }

    override fun updateJointManualPosition(newJointPosition: Angle) {
        jointManualTargetPosition.mut_replace(newJointPosition)
    }

    override fun updateJointMotorsControlGains(slot: Int) {
        val validatedSlot = MathUtil.clamp(slot, 0, 2) // Make sure the selected slot is either 0, 1, or 2
        // Clone the initial config
        val newMotorsConfig = JointConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> {
                newMotorsConfig.Slot0 = SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> { // Secondary not declared
                newMotorsConfig.Slot1 = SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorController.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun setJointManualPosition(): Runnable {
        return {
            jointTargetPosition.mut_replace(jointManualTargetPosition)

            leadMotorController.angularSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.JOINT_CONTROL_TYPE,
                jointManualTargetPosition,
                SubsystemsMovementLimits.JOINT_POSITION_LIMITS,
                JointConstants.Mechanical.REDUCTION,
                Optional.of(SubsystemsMotionTargets.JOINT_SECONDARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun setJointPosition(jointPosition: Angle): Runnable {
        return {
            jointTargetPosition.mut_replace(jointPosition)

            leadMotorController.angularSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.JOINT_CONTROL_TYPE,
                jointPosition,
                SubsystemsMovementLimits.JOINT_POSITION_LIMITS,
                JointConstants.Mechanical.REDUCTION,
                Optional.of(SubsystemsMotionTargets.JOINT_PRIMARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun stopJoint(): Runnable {
        return { leadMotorController.stopMotor() }
    }

    override fun coastJointMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorController.coast()
        }
    }

    override fun brakeJointMotors(): Runnable {
        return {
            leadMotorController.brake()
            followerMotorController.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(JointConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorController.applyConfigAndClearFaults(JointConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorController.follow(
            leadMotorController.getMotorInstance(),
            JointConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}