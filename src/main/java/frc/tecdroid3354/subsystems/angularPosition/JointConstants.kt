package frc.tecdroid3354.subsystems.angularPosition

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
import edu.wpi.first.units.measure.MomentOfInertia
import frc.tecdroid3354.constants.CanBuses
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.constants.SimConstants
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.devices.KrakenMotors
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.mechanical.Reduction
import java.util.Optional
import kotlin.math.pow

object JointConstants {
    /**
     * Contains the ID Of any hardware related to the subsystem and the CANBUS it is on
     */
    object Identification {
        const val JOINT_CANBUS_NAME: String = CanBuses.RIO_CANBUS
        const val LEAD_MOTOR_ID: Int = 20
        const val FOLLOWER_MOTOR_ID: Int = 21
    }

    /**
     * Only for gear ratio ([Reduction]). In the case of linear subsystems, the sprocket also goes here.
     */
    object Mechanical {
        val REDUCTION: Reduction = Reduction(360.0)

        const val NUMBER_OF_MOTORS: Int = 2

        // From OnShape, accounting for the elevator of Botzilla (2025) as of 26/07/2026
        private val MECHANISM_INERTIA: MomentOfInertia = (1684.1562.times(SimConstants.FREEDOM_UNITS_TO_METRIC_MOI)).kilogramSquareMeters

        // From mechanism perspective
        // Check: https://www.motioncontroltips.com/how-do-gearmotors-impact-reflected-mass-inertia-from-the-load/
        // which refers to the same formula but from motor perspective (reflected load inertia)
        val MOMENT_OF_INERTIA: MomentOfInertia =
            MECHANISM_INERTIA
                .plus(
                    SimConstants.ESTIMATED_MOTOR_MOI
                        .times(NUMBER_OF_MOTORS.toDouble())
                        .times(REDUCTION.getRatio().pow(2))
                )
    }

    /**
     * Contains initial configuration for the Joint motors assuming Phoenix API.
     * Configurations meant to be tunable live, limits, control gains, motion targets and movement presets
     * are all stored in a separate file where they are next to those of all other subsystems (excluding drivetrain).
     * This structure is to have a single file that is regularly consulted by the Software Team, whereas this one
     * remains mostly untouched unless the Design or Electrical Teams change something.
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
            Optional.of<Slot0Configs>(SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.JOINT_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureAngularMotionMagic(
                    SubsystemsMotionTargets.JOINT_PRIMARY_MOTION_TARGETS,
                    Mechanical.REDUCTION))
        )
    }

    /**
     * Merely contains the folder names for different Telemetry tabs.
     *
     * **NOTE:** All alerts share a common parent folder defined in [RobotTelemetry.CONNECTION_ALERTS_TAB] for
     * easier alert visualization in Elastic.
     */
    object Telemetry {
        const val SUBSYSTEM_TAB                         : String = "Joint"
        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"
        const val FOLLOWER_MOTOR_INPUTS_TAB             : String = "${SUBSYSTEM_TAB}/Follower Motor"

        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"

        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.JOINT_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
        const val FOLLOWER_MOTOR_CONNECTION_ALERT_TAB   : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.JOINT_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.FOLLOWER_MOTOR_ID}"
    }
}
