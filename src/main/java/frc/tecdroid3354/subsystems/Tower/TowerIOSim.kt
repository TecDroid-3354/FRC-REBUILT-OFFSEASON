package frc.tecdroid3354.subsystems.Tower

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

class TowerIOSim : TowerIO {
    private val subsystemSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(TowerConstants.Mechanical.NUMBER_OF_MOTORS),
            TowerConstants.Mechanical.MOMENT_OF_INERTIA.kilogramSquareMeters,
            TowerConstants.Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(TowerConstants.Mechanical.NUMBER_OF_MOTORS),
    )

    private val leadMotorReal: OpTalonFX = OpTalonFX(
        TowerConstants.Identification.LEAD_MOTOR_ID,
        TowerConstants.Identification.TOWER_CANBUS_NAME)
    private val followerMotorReal: OpTalonFX = OpTalonFX(
        TowerConstants.Identification.FOLLOWER_MOTOR_ID,
        TowerConstants.Identification.TOWER_CANBUS_NAME)

    private val leadMotorSim: TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim: TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(TowerConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    /**
     * Note that [towerVelocityTarget] may contain the same value as [manualTowerVelocityTarget] when
     * [enableTowerManualVelocity] is commanded.
     */
    private val manualTowerVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)
    private val towerVelocityTarget: MutAngularVelocity = DegreesPerSecond.mutable(0.0)

    override fun updateTowerInputs(inputs: TowerIO.TowerIOInputs,
                                   leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        //
        // START PHYSICS UPDATE
        //
        val appliedVolts = leadMotorSim.motorVoltage

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        val motorVelocity: AngularVelocity = leadMotorReal.getAngularSubsystemToMotorVelocity(
            subsystemSim.angularVelocity,
            TowerConstants.Mechanical.REDUCTION)
        val motorAcceleration: AngularAcceleration = leadMotorReal.getAngularSubsystemToMotorAcceleration(
            subsystemSim.angularAcceleration,
            TowerConstants.Mechanical.REDUCTION
        )

        leadMotorSim.setRotorVelocity(motorVelocity)
        leadMotorSim.setRotorAcceleration(motorAcceleration)

        followerMotorSim.setRotorVelocity(if (inverseMotorReading) motorVelocity.unaryMinus() else motorVelocity)
        followerMotorSim.setRotorAcceleration(if (inverseMotorReading) motorAcceleration.unaryMinus() else motorAcceleration)

        //
        // END PHYSICS UPDATE
        //

        inputs.towerActualVelocity.mut_replace(subsystemSim.angularVelocity)
        inputs.towerTargetVelocity.mut_replace(towerVelocityTarget)
        inputs.towerManualTargetVelocity.mut_replace(manualTowerVelocityTarget)
        inputs.towerPresetVelocity.mut_replace(SubsystemsPresetTargets.TOWER_PRESET_RPM)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
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

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)
    }

    override fun enableTowerManualVelocity(): Runnable {
        return {
            towerVelocityTarget.mut_replace(manualTowerVelocityTarget) // Update target velocity

            leadMotorReal.angularSubsystemVelocityRequest(
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

            leadMotorReal.angularSubsystemVelocityRequest(
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
            leadMotorReal.stopMotor()
        }
    }

    override fun coastTowerMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeTowerMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(TowerConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(TowerConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            TowerConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}
