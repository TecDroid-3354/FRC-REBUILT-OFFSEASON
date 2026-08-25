package frc.tecdroid3354.subsystems.angularVelocity

import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.seconds

class FlywheelIOSim : FlywheelIO {
    private val subsystemSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(FlywheelConstants.Mechanical.NUMBER_OF_MOTORS),
            FlywheelConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            FlywheelConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(FlywheelConstants.Mechanical.NUMBER_OF_MOTORS),
    )

    private val leadMotorReal: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.LEAD_MOTOR_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)
    private val followerMotorReal: OpTalonFX = OpTalonFX(
        FlywheelConstants.Identification.FOLLOWER_MOTOR_ID,
        FlywheelConstants.Identification.FLYWHEEL_CANBUS_NAME)

    private val leadMotorSim: TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim: TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(FlywheelConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    /**
     * Note that [flywheelVelocityTarget] may contain the same value as [manualFlywheelVelocityTarget] when
     * [enableFlywheelManualVelocity] is commanded.
     */
    private val manualFlywheelVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val flywheelVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateFlywheelInputs(inputs: FlywheelIO.FlywheelIOInputs,
                                      leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorVelocity: AngularVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.angularVelocity,
            FlywheelConstants.Mechanical.REDUCTION)
        val motorAcceleration: AngularAcceleration = leadMotorReal.getAngularSubsystemToMotorAcceleration(
            subsystemSim.angularAcceleration,
            FlywheelConstants.Mechanical.REDUCTION
        )

        leadMotorSim.setRotorVelocity(motorVelocity)
        leadMotorSim.setRotorAcceleration(motorAcceleration)

        followerMotorSim.setRotorVelocity(if (inverseMotorReading) motorVelocity.unaryMinus() else motorVelocity)
        followerMotorSim.setRotorAcceleration(if (inverseMotorReading) motorAcceleration.unaryMinus() else motorAcceleration)

        //
        // END PHYSICS UPDATE
        //

        inputs.flywheelActualVelocity.mut_replace(subsystemSim.angularVelocity)
        inputs.flywheelTargetVelocity.mut_replace(flywheelVelocityTarget)
        inputs.flywheelManualTargetVelocity.mut_replace(manualFlywheelVelocityTarget)
        inputs.flywheelPresetVelocity.mut_replace(SubsystemsPresetTargets.FLYWHEEL_PRESET_RPM)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
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
            1 -> {
                newMotorsConfig.Slot1 = SubsystemsControlGains.FLYWHEEL_MOTOR_SECONDARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableFlywheelManualVelocity(): Runnable {
        return {
            flywheelVelocityTarget.mut_replace(manualFlywheelVelocityTarget) // Update target velocity

            leadMotorReal.angularSubsystemVelocityRequest(
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

            leadMotorReal.angularSubsystemVelocityRequest(
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

            leadMotorReal.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.FLYWHEEL_CONTROL_TYPE,
                flywheelCalculatedVelocity,
                SubsystemsMovementLimits.FLYWHEEL_VELOCITY_LIMITS,
                FlywheelConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopFlywheel(): Runnable {
        return {
            leadMotorReal.stopMotor()
        }
    }

    override fun coastFlywheelMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeFlywheelMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(FlywheelConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            FlywheelConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}
