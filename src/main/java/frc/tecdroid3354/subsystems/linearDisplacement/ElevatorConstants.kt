package frc.tecdroid3354.subsystems.linearDisplacement

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.tecdroid3354.constants.CanBuses
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.subsystems.angularPosition.JointConstants
import frc.tecdroid3354.utils.mechanical.Reduction
import frc.tecdroid3354.utils.Sprocket
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.devices.KrakenMotors
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.kilograms
import frc.tecdroid3354.utils.meters
import java.util.Optional

/**
 * Intended to contain ALL values that will not change without manual manipulation.
 * Separated into different objects to categorize.
 */
object ElevatorConstants {
    /**
     * All constants for hardware identification
     */
    object Identification {
        const val ELEVATOR_CANBUS_NAME: String = CanBuses.CANIVORE_CANBUS
        const val LEAD_MOTOR_ID = 30
        const val FOLLOWER_MOTOR_ID = 31
    }
    /**
     * All constants that have physical contact with the elevator
     * All values are placeholders and must be tuned for your specific robot.
     */
    object Mechanical {
        val REDUCTION                   : Reduction = Reduction(8.9285)                    // Gear ratio motor - elevator
        const val NUMBER_OF_MOTORS      : Int = 2
        val SPROCKET                    : Sprocket = Sprocket.fromRadius((1.0 + 1.0 / 8.0).inches)    // Rotational -> Linear Motion
        val MASS                        : Mass = 8.0.kilograms                          // Simulation purposes
    }

    /**
     * Stores all initial configuration of the subsystem motor(s) assuming they are accessed through the
     * Phoenix API.
     *
     * In case you subsystem motors use RevLib or other API, it must be specified in the object name.
     */
    object PhoenixMotorConfiguration {
        val followerMotorAlignment: MotorAlignmentValue = MotorAlignmentValue.Aligned

        private val neutralMode: NeutralModeValue = NeutralModeValue.Brake
        private val motorDirection: InvertedValue = InvertedValue.CounterClockwise_Positive

        private val supplyCurrentLimit: Current = 30.0.amps
        private val statorCurrentLimit: Current = 80.0.amps

        val initialMotorsConfiguration: TalonFXConfiguration = KrakenMotors.createTalonFXConfiguration(
            Optional.of<MotorOutputConfigs>(
                KrakenMotors.configureMotorOutputs(neutralMode, motorDirection)
            ),
            Optional.of<CurrentLimitsConfigs>(
                KrakenMotors.configureCurrentLimits(supplyCurrentLimit, statorCurrentLimit)
            ),
            // NOTE: Since only primary gains are declared, all slot configs share it.
            // Motion is altered through MotionTargets in this example, which uses the same SlotConfigs
            // but with different cruise velocity, acceleration and jerk.
            Optional.of<Slot0Configs>(SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureLinearMotionMagic(
                    SubsystemsMotionTargets.ELEVATOR_PRIMARY_MOTION_TARGETS, // May be changed through dynamic requests
                    Mechanical.REDUCTION, Mechanical.SPROCKET))
        )
    }

    /**
     * Stores the tab name for all elevator fields. This includes the tab + message for alerts.
     */
    object Telemetry {
        const val SUBSYSTEM_TAB                         : String = "Elevator"
        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"
        const val FOLLOWER_MOTOR_INPUTS_TAB             : String = "${SUBSYSTEM_TAB}/Follower Motor"
        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"
        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.ELEVATOR_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
        const val FOLLOWER_MOTOR_CONNECTION_ALERT_TAB   : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.ELEVATOR_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.FOLLOWER_MOTOR_ID}"

        const val SUBSYSTEM_VISUALIZATION_2D_TAB        : String =
            "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB}/${SUBSYSTEM_TAB}"
        const val SUBSYSTEM_VISUALIZATION_3D_TAB        : String =
            "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_3D_TAB}/${SUBSYSTEM_TAB}"
    }
}