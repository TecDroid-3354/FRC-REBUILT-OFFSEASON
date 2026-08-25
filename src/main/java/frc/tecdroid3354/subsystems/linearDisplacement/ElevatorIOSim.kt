package frc.tecdroid3354.subsystems.linearDisplacement

import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorConstants.Mechanical
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.utils.kilograms
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.seconds
import org.littletonrobotics.junction.Logger
import java.util.Optional

/**
 * Simulation layer for the [ElevatorSubsystem]. You will notice that all control methods are **EXACTLY**
 * equal to those of [ElevatorIOTalonFX], that is on purpose due to how simulation between WPILIB Physics Simulation
 * and Phoenix6 API works and to better reflect the behaviour of the real subsystem.
 *
 * The only difference in this layer is that we retrieve the [TalonFXSimState] from each subsystem motor, which get
 * the output voltage reading of the calculations inside the "real" motor so that we can update the
 * WPILIB Physics Simulation.
 *
 * All updates are done through the updateInputs method, so that it's called periodically (which is a **MUST**)
 * inside [ElevatorSubsystem].
 */
class ElevatorIOSim: ElevatorIO {
    private val subsystemSim: ElevatorSim = ElevatorSim( // WPILIB Physics Simulation
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60Foc(Mechanical.NUMBER_OF_MOTORS), Mechanical.MASS.kilograms,
            Mechanical.SPROCKET.radius.meters, Mechanical.REDUCTION.getRatio()
        ),
        DCMotor.getKrakenX60Foc(Mechanical.NUMBER_OF_MOTORS),
        (SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS.minimum as Distance).meters,
        (SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS.maximum as Distance).meters,
        true,
        SubsystemsPresetTargets.ELEVATOR_HOME_DISPLACEMENT.meters,
    )

    //
    // NOTE: Real motors are declared and configured, since these are the ones to be commanded so that
    //      their simStates get the calculated outputs, which are then passed to the subsystemSim.
    //
    //      This will not interfere with the hardware layer, since this class is only called during simulation,
    //      which should be used while connected to the robot.
    //
    private val leadMotorReal: OpTalonFX =
        OpTalonFX(ElevatorConstants.Identification.LEAD_MOTOR_ID, ElevatorConstants.Identification.ELEVATOR_CANBUS_NAME)
    private val followerMotorReal: OpTalonFX =
        OpTalonFX(ElevatorConstants.Identification.FOLLOWER_MOTOR_ID, ElevatorConstants.Identification.ELEVATOR_CANBUS_NAME)

    // It is important that the Sim Motors come directly from real motors
    private val leadMotorSim: TalonFXSimState = leadMotorReal.getMotorInstance().simState
    private val followerMotorSim: TalonFXSimState = followerMotorReal.getMotorInstance().simState

    private val inverseMotorReading: Boolean = when(ElevatorConstants.PhoenixMotorConfiguration.followerMotorAlignment) {
        MotorAlignmentValue.Aligned -> false
        MotorAlignmentValue.Opposed -> true
    }

    private val elevatorTargetDisplacement      : MutDistance = Meters.mutable(0.0)
    private val elevatorManualTargetDisplacement: MutDistance = Meters.mutable(0.0)

    override fun updateElevatorInputs(inputs: ElevatorIO.ElevatorIOInputs,
                                      leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        //
        // START OF PHYSICS UPDATE
        //

        val appliedVolts = leadMotorSim.motorVoltage // Gotten from the "real" motor calculations

        subsystemSim.setInputVoltage(appliedVolts)
        subsystemSim.update(RobotConstants.LOOP_TIME.seconds)

        // Real motors are only accessed for their auxiliary mechanism -> motor units here
        val motorPosition: Angle = leadMotorReal.getLinearSubsystemToMotorPosition(
            subsystemSim.positionMeters.meters,
            Mechanical.REDUCTION,
            Mechanical.SPROCKET
        )
        val motorVelocity: AngularVelocity = leadMotorReal.getLinearSubsystemToMotorVelocity(
            subsystemSim.velocityMetersPerSecond.metersPerSecond,
            Mechanical.REDUCTION,
            Mechanical.SPROCKET
        )

        leadMotorSim.setRawRotorPosition(motorPosition)
        leadMotorSim.setRotorVelocity(motorVelocity)

        followerMotorSim.setRawRotorPosition(if (inverseMotorReading) motorPosition.unaryMinus() else motorPosition)
        followerMotorSim.setRotorVelocity(if (inverseMotorReading) motorVelocity.unaryMinus() else motorVelocity)

        //
        // END OF PHYSICS UPDATE
        //

        inputs.elevatorDisplacement.mut_replace(subsystemSim.positionMeters.meters)
        inputs.elevatorTargetDisplacement.mut_replace(elevatorTargetDisplacement)
        inputs.elevatorManualTargetDisplacement.mut_replace(elevatorManualTargetDisplacement)

        leadMotorReal.updateInputs(leadMotorInputs)
        followerMotorReal.updateInputs(followerMotorInputs)
    }

    override fun updateElevatorManualDisplacement(newElevatorManualDisplacement: Distance) {
        this.elevatorManualTargetDisplacement.mut_replace(newElevatorManualDisplacement)
    }

    override fun updateElevatorMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> { // Note that only primary gains are declared, hence is the only one in use for all slots
                newMotorsConfig.Slot0 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> {
                newMotorsConfig.Slot1 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, since we're using the validatedSlot
                newMotorsConfig.Slot2 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorReal.applyConfigAndClearFaults(newMotorsConfig)
        followerMotorReal.applyConfigAndClearFaults(newMotorsConfig)

        Logger.recordOutput("Elevator/Recorded Slot0 kP", newMotorsConfig.Slot0.kP)
        Logger.recordOutput("Elevator/Recorded Slot0 kI", newMotorsConfig.Slot0.kI)
        Logger.recordOutput("Elevator/Recorded Slot0 kD", newMotorsConfig.Slot0.kD)
        Logger.recordOutput("Elevator/Recorded Slot0 kS", newMotorsConfig.Slot0.kS)
        Logger.recordOutput("Elevator/Recorded Slot0 kV", newMotorsConfig.Slot0.kV)
        Logger.recordOutput("Elevator/Recorded Slot0 kA", newMotorsConfig.Slot0.kA)
        Logger.recordOutput("Elevator/Recorded Slot0 kG", newMotorsConfig.Slot0.kG)
    }

    override fun setElevatorManualTargetDisplacement(): Runnable {
        return {
            this.elevatorTargetDisplacement.mut_replace(elevatorManualTargetDisplacement)

            leadMotorReal.linearSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.ELEVATOR_CONTROL_TYPE,
                elevatorManualTargetDisplacement,
                SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS,
                Mechanical.SPROCKET,
                Mechanical.REDUCTION,
                // Note how the SECONDARY motion targets are used when using manual target displacement
                Optional.of(SubsystemsMotionTargets.ELEVATOR_SECONDARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun setElevatorTargetDisplacement(elevatorTargetDisplacement: Distance): Runnable {
        return {
            this.elevatorTargetDisplacement.mut_replace(elevatorTargetDisplacement)

            leadMotorReal.linearSubsystemPositionDynamicRequest(
                SubsystemsControlRequests.ELEVATOR_CONTROL_TYPE,
                elevatorTargetDisplacement,
                SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS,
                Mechanical.SPROCKET,
                Mechanical.REDUCTION,
                Optional.of(SubsystemsMotionTargets.ELEVATOR_PRIMARY_MOTION_TARGETS),
                Optional.empty(), Optional.empty()
            )
        }
    }

    override fun stopElevator(): Runnable {
        return { leadMotorReal.stopMotor() }
    }

    override fun coastElevatorMotors(): Runnable {
        return {
            leadMotorReal.coast()
            followerMotorReal.coast()
        }
    }

    override fun brakeElevatorMotors(): Runnable {
        return {
            leadMotorReal.brake()
            followerMotorReal.brake()
        }
    }

    override fun initialMotorConfiguration() {
        leadMotorReal.applyConfigAndClearFaults(ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorReal.applyConfigAndClearFaults(ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorReal.follow(
            leadMotorReal.getMotorInstance(),
            ElevatorConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }


}