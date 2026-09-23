package frc.tecdroid3354.subsystems.IntakeDeploy

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
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Mass
import frc.tecdroid3354.constants.CanBuses
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.utils.mechanical.Reduction
import frc.tecdroid3354.utils.Sprocket
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.devices.KrakenMotors
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.kilograms
import java.util.Optional

/**
 * Intended to contain ALL values that will not change without manual manipulation.
 * Separated into different objects to categorize.
 */
object IntakeDeployConstants {
    /**
     * All constants for hardware identification
     */
    object Identification {
        const val INTAKE_DEPLOY_CANBUS_NAME: String = CanBuses.CANIVORE_CANBUS
        const val LEAD_MOTOR_ID = 23
    }
    /**
     * All constants that have physical contact with the subsystem
     * All values are placeholders and must be tuned for your specific robot.
     */
    object Mechanical {
        val REDUCTION                   : Reduction = Reduction((56.0 / 9.0) * (50.0 / 28.0))                    // Gear ratio motor - subsystem
        const val NUMBER_OF_MOTORS      : Int = 1
        val SPROCKET                    : Sprocket = Sprocket.fromRadius((1.5).inches)    // Rotational -> Linear Motion
        val MASS                        : Mass = 6.4.kilograms                          // Simulation purposes
    }

    /**
     * Stores all initial configuration of the subsystem motor(s) assuming they are accessed through the
     * Phoenix API.
     *
     * In case you subsystem motors use RevLib or other API, it must be specified in the object name.
     */
    object PhoenixMotorConfiguration {
        private val neutralMode: NeutralModeValue = NeutralModeValue.Coast
        private val motorDirection: InvertedValue = InvertedValue.CounterClockwise_Positive

        private val supplyCurrentLimit: Current = 40.0.amps
        private val statorCurrentLimit: Current = 75.0.amps

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
            Optional.of<Slot0Configs>(SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureLinearMotionMagic(
                    SubsystemsMotionTargets.INTAKE_DEPLOY_PRIMARY_MOTION_TARGETS, // May be changed through dynamic requests
                    Mechanical.REDUCTION, Mechanical.SPROCKET))
        )
    }

    /**
     * Stores the tab name for all subsystem fields. This includes the tab + message for alerts.
     */
    object Telemetry {
        const val SUBSYSTEM_TAB                         : String = "Intake Deploy"
        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"
        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"

        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.INTAKE_DEPLOY_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
    }
}