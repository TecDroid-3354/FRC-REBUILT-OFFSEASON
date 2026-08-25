package frc.tecdroid3354.subsystems.angularPosition

import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotDimensions
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelConstants
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.radians
import frc.tecdroid3354.utils.radiansPerSecond
import frc.tecdroid3354.utils.seconds
import java.util.Optional

class JointIOSim: JointIO {
    private val subsystemSim: SingleJointedArmSim = SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60Foc(JointConstants.Mechanical.NUMBER_OF_MOTORS),
            JointConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            JointConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(JointConstants.Mechanical.NUMBER_OF_MOTORS),
        JointConstants.Mechanical.REDUCTION.getRatio(),
        RobotDimensions.ELEVATOR_MINIMUM_LENGTH.meters,
        (SubsystemsMovementLimits.JOINT_POSITION_LIMITS.minimum as Angle).radians,
        (SubsystemsMovementLimits.JOINT_POSITION_LIMITS.maximum as Angle).radians,
        true,
        SubsystemsPresetTargets.JOINT_HOME_ANGLE.radians
    )

    private val leadMotorReal     : OpTalonFX = OpTalonFX(
        JointConstants.Identification.LEAD_MOTOR_ID, JointConstants.Identification.JOINT_CANBUS_NAME
    )
    private val followerMotorReal : OpTalonFX = OpTalonFX(
        JointConstants.Identification.FOLLOWER_MOTOR_ID, JointConstants.Identification.JOINT_CANBUS_NAME
    )

    private val leadMotorSim        : TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim    : TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(FlywheelConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    private val jointTargetPosition: MutAngle = Degrees.mutable(0.0)
    private val jointManualTargetPosition: MutAngle = Degrees.mutable(0.0)

    @Suppress("DuplicatedCode")
    override fun updateJointInputs(inputs: JointIO.JointIOInputs,
                                   leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs)  {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorPosition = leadMotorReal.getAngularSubsystemToMotorPosition(
            subsystemSim.angleRads.radians,
            JointConstants.Mechanical.REDUCTION
        )
        val motorVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.velocityRadPerSec.radiansPerSecond,
            JointConstants.Mechanical.REDUCTION
        )
        val motorPositionFollower = if (inverseMotorReading) motorPosition.unaryMinus() else motorPosition
        val motorVelocityFollower = if (inverseMotorReading) motorVelocity.unaryMinus() else motorVelocity

        leadMotorSim.setRawRotorPosition(motorPosition)
        leadMotorSim.setRotorVelocity(motorVelocity)

        followerMotorSim.setRawRotorPosition(motorPositionFollower)
        followerMotorSim.setRotorVelocity(motorVelocityFollower)
        //
        // END PHYSICS UPDATE
        //
        inputs.jointActualPosition.mut_replace(subsystemSim.angleRads.radians)
        inputs.jointTargetPosition.mut_replace(jointTargetPosition)
        inputs.jointManualTargetPosition.mut_replace(jointManualTargetPosition)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
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
                newMotorsConfig.Slot2 = SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun setJointManualPosition(): Runnable {
        return {
            jointTargetPosition.mut_replace(jointManualTargetPosition)

            leadMotorReal.angularSubsystemPositionDynamicRequest(
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

            leadMotorReal.angularSubsystemPositionDynamicRequest(
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
        return { leadMotorReal.stopMotor() }
    }

    override fun coastJointMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeJointMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(JointConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(JointConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            JointConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}