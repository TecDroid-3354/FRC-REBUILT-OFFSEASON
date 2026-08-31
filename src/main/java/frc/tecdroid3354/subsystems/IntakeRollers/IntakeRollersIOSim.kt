package frc.tecdroid3354.subsystems.IntakeRollers

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
import frc.tecdroid3354.constants.*
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.rotationsPerMinute
import frc.tecdroid3354.utils.seconds

class IntakeRollersIOSim : IntakeRollersIO {
    private val subsystemSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(IntakeRollersConstants.Mechanical.NUMBER_OF_MOTORS),
            IntakeRollersConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            IntakeRollersConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(IntakeRollersConstants.Mechanical.NUMBER_OF_MOTORS),
    )

    private val leadMotorReal: OpTalonFX = OpTalonFX(
        IntakeRollersConstants.Identification.LEAD_MOTOR_ID,
        IntakeRollersConstants.Identification.INTAKE_ROLLERS_CANBUS_NAME)
    private val followerMotorReal: OpTalonFX = OpTalonFX(
        IntakeRollersConstants.Identification.FOLLOWER_MOTOR_ID,
        IntakeRollersConstants.Identification.INTAKE_ROLLERS_CANBUS_NAME)

    private val leadMotorSim: TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim: TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(IntakeRollersConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    /**
     * Note that [intakeRollersVelocityTarget] may contain the same value as [manualIntakeRollersVelocityTarget] when
     * [enableIntakeRollersManualVelocity] is commanded.
     */
    private val manualIntakeRollersVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val intakeRollersVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateIntakeRollersInputs(inputs: IntakeRollersIO.IntakeRollersIOInputs,
                                           leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorVelocity: AngularVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.angularVelocity,
            IntakeRollersConstants.Mechanical.REDUCTION)
        val motorAcceleration: AngularAcceleration = leadMotorReal.getAngularSubsystemToMotorAcceleration(
            subsystemSim.angularAcceleration,
            IntakeRollersConstants.Mechanical.REDUCTION
        )

        leadMotorSim.setRotorVelocity(motorVelocity)
        leadMotorSim.setRotorAcceleration(motorAcceleration)

        followerMotorSim.setRotorVelocity(if (inverseMotorReading) motorVelocity.unaryMinus() else motorVelocity)
        followerMotorSim.setRotorAcceleration(if (inverseMotorReading) motorAcceleration.unaryMinus() else motorAcceleration)

        //
        // END PHYSICS UPDATE
        //

        inputs.intakeRollersActualVelocity.mut_replace(subsystemSim.angularVelocity)
        inputs.intakeRollersTargetVelocity.mut_replace(intakeRollersVelocityTarget)
        inputs.intakeRollersManualTargetVelocity.mut_replace(manualIntakeRollersVelocityTarget)
        inputs.intakeRollersPresetVelocity.mut_replace(SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
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
            else -> { // Can assume else {} branch to be 2, but defaults to primary since tertiary are not declared.
                newMotorsConfig.Slot2 = SubsystemsControlGains.INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableIntakeRollersManualVelocity(): Runnable {
        return {
            intakeRollersVelocityTarget.mut_replace(manualIntakeRollersVelocityTarget) // Update target velocity

            leadMotorReal.angularSubsystemVelocityRequest(
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

            leadMotorReal.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.INTAKE_ROLLERS_CONTROL_TYPE,
                SubsystemsPresetTargets.INTAKE_ROLLERS_PRESET_RPM,
                SubsystemsMovementLimits.INTAKE_ROLLERS_VELOCITY_LIMITS,
                IntakeRollersConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopIntakeRollers(): Runnable {
        return {
            intakeRollersVelocityTarget.mut_replace(0.0.rotationsPerMinute)
            leadMotorReal.stopMotor()
        }
    }

    override fun coastIntakeRollersMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeIntakeRollersMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(IntakeRollersConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(IntakeRollersConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            IntakeRollersConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}
