package frc.tecdroid3354.subsystems.Hopper

import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.tecdroid3354.constants.*
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.kilograms
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.rotationsPerMinute
import frc.tecdroid3354.utils.seconds

class HopperIOSim : HopperIO {
    private val subsystemSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(HopperConstants.Mechanical.NUMBER_OF_MOTORS),
            HopperConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            HopperConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(HopperConstants.Mechanical.NUMBER_OF_MOTORS),
    )

    private val leadMotorReal: OpTalonFX = OpTalonFX(
        HopperConstants.Identification.LEAD_MOTOR_ID,
        HopperConstants.Identification.HOPPER_CANBUS_NAME)

    private val leadMotorSim: TalonFXSimState = leadMotorReal.getMotorInstance().simState

    /**
     * Note that [hopperVelocityTarget] may contain the same value as [manualHopperVelocityTarget] when
     * [enableHopperManualVelocity] is commanded.
     */
    private val manualHopperVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val hopperVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateHopperInputs(inputs: HopperIO.HopperIOInputs, leadMotorInputs: MotorIO.MotorIOInputs) {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorVelocity: AngularVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.angularVelocity,
            HopperConstants.Mechanical.REDUCTION)
        val motorAcceleration: AngularAcceleration = leadMotorReal.getAngularSubsystemToMotorAcceleration(
            subsystemSim.angularAcceleration,
            HopperConstants.Mechanical.REDUCTION
        )

        leadMotorSim.setRotorVelocity(motorVelocity)
        leadMotorSim.setRotorAcceleration(motorAcceleration)

        //
        // END PHYSICS UPDATE
        //

        inputs.hopperActualVelocity.mut_replace(subsystemSim.angularVelocity)
        inputs.hopperTargetVelocity.mut_replace(hopperVelocityTarget)
        inputs.hopperManualTargetVelocity.mut_replace(manualHopperVelocityTarget)
        inputs.hopperPresetVelocity.mut_replace(SubsystemsPresetTargets.HOPPER_PRESET_RPM)

        leadMotorReal.updateInputs(leadMotorInputs)
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

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableHopperManualVelocity(): Runnable {
        return {
            hopperVelocityTarget.mut_replace(manualHopperVelocityTarget) // Update target velocity

            leadMotorReal.angularSubsystemVelocityRequest(
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

            leadMotorReal.angularSubsystemVelocityRequest(
                SubsystemsControlRequests.HOPPER_CONTROL_TYPE,
                SubsystemsPresetTargets.HOPPER_PRESET_RPM,
                SubsystemsMovementLimits.HOPPER_VELOCITY_LIMITS,
                HopperConstants.Mechanical.REDUCTION
            )
        }
    }

    override fun stopHopper(): Runnable {
        return {
            hopperVelocityTarget.mut_replace(0.0.rotationsPerMinute)
            leadMotorReal.stopMotor()
        }
    }

    override fun coastHopperMotors(): Runnable {
        return {
            leadMotorReal.coast()
        }
    }

    override fun brakeHopperMotors(): Runnable {
        return {
            leadMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(HopperConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
    }
}
