package frc.tecdroid3354.subsystems.Hood

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
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.subsystems.Flywheel.FlywheelConstants
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.radians
import frc.tecdroid3354.utils.radiansPerSecond
import frc.tecdroid3354.utils.seconds

class HoodIOSim: HoodIO {
    private val subsystemSim: SingleJointedArmSim = SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX44Foc(HoodConstants.Mechanical.NUMBER_OF_MOTORS),
            HoodConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            HoodConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX44Foc(HoodConstants.Mechanical.NUMBER_OF_MOTORS),
        HoodConstants.Mechanical.REDUCTION.getRatio(),
        RobotDimensions.INTAKE_MINIMUM_LENGTH.meters,
        (SubsystemsMovementLimits.HOOD_POSITION_LIMITS.minimum as Angle).radians,
        (SubsystemsMovementLimits.HOOD_POSITION_LIMITS.maximum as Angle).radians,
        true,
        SubsystemsPresetTargets.HOOD_HOME_ANGLE.radians
    )

    private val leadMotorReal     : OpTalonFX = OpTalonFX(
        HoodConstants.Identification.LEAD_MOTOR_ID, HoodConstants.Identification.HOOD_CANBUS_NAME
    )
    private val followerMotorReal : OpTalonFX = OpTalonFX(
        HoodConstants.Identification.FOLLOWER_MOTOR_ID, HoodConstants.Identification.HOOD_CANBUS_NAME
    )

    private val leadMotorSim        : TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim    : TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(FlywheelConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    private val hoodTargetPosition: MutAngle = Degrees.mutable(0.0)
    private val hoodManualTargetPosition: MutAngle = Degrees.mutable(0.0)

    @Suppress("DuplicatedCode")
    override fun updateHoodInputs(inputs: HoodIO.HoodIOInputs,
                                  leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs)  {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorPosition = leadMotorReal.getAngularSubsystemToMotorPosition(
            subsystemSim.angleRads.radians,
            HoodConstants.Mechanical.REDUCTION
        )
        val motorVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.velocityRadPerSec.radiansPerSecond,
            HoodConstants.Mechanical.REDUCTION
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
        inputs.hoodActualPosition.mut_replace(subsystemSim.angleRads.radians)
        inputs.hoodTargetPosition.mut_replace(hoodTargetPosition)
        inputs.hoodManualTargetPosition.mut_replace(hoodManualTargetPosition)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
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
                newMotorsConfig.Slot2 = SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun setHoodManualPosition(): Runnable {
        return {
            hoodTargetPosition.mut_replace(hoodManualTargetPosition)

            leadMotorReal.angularSubsystemPositionRequest(
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

            leadMotorReal.angularSubsystemPositionRequest(
                SubsystemsControlRequests.HOOD_CONTROL_TYPE,
                hoodPosition,
                SubsystemsMovementLimits.HOOD_POSITION_LIMITS,
                HoodConstants.Mechanical.REDUCTION,
            )
        }
    }

    override fun stopHood(): Runnable {
        return { leadMotorReal.stopMotor() }
    }

    override fun coastHoodMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeHoodMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(HoodConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(HoodConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            HoodConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}