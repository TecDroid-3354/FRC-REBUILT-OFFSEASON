package frc.tecdroid3354.subsystems.Flywheel

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
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.tecdroid3354.constants.CanBuses
import frc.tecdroid3354.constants.RobotTelemetry
import frc.tecdroid3354.constants.SimConstants
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.controlProfiles.Polynomials
import frc.tecdroid3354.utils.devices.KrakenMotors
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.mechanical.Reduction
import java.util.Optional
import kotlin.math.pow

object FlywheelConstants {
    /**
     * Contains the ID Of any hardware related to the subsystem and the CANBUS it is on
     */
    object Identification {
        const val FLYWHEEL_CANBUS_NAME: String = CanBuses.RIO_CANBUS
        const val LEAD_MOTOR_ID: Int = 40
        const val FOLLOWER_LEFT_ID: Int = 41
        const val FOLLOWER_RIGHT_ONE_ID: Int = 42
        const val FOLLOWER_RIGHT_TWO_ID: Int = 43
    }

    /**
     * Only for gear ratio ([Reduction]) and MOI (for simulation)
     * In the case of linear subsystems, the sprocket also goes here.
     */
    object Mechanical {
        val REDUCTION: Reduction = Reduction(1.4) // Motor -> Flywheel (0.56), Motor -> Roller (1.4)
        val ROLLER_DIAMETER: Distance = 3.0.inches

        const val ESTIMATED_ROLLER_EFFICIENCY: Double = 0.95

        const val NUMBER_OF_MOTORS: Int = 4

        // From OnShape, accounting for the main roller and flywheel of Tutankabot as of 26/08/2026
        private val MECHANISM_INERTIA: MomentOfInertia = (67.794767.times(SimConstants.FREEDOM_UNITS_TO_METRIC_MOI)).kilogramSquareMeters

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
     * Only in the scenario you use a polynomial to calculate the target flywheel velocity.
     * In this case, Time of Flight (TOF) polynomials are also included.
     *
     * All polynomials were calculated from TecDroid's Trajectory Simulator.
     */
    object StoredPolynomials {
        val SCORING_POLYNOMIAL          : Result<Polynomials> = Polynomials.of(degree = 3,
            -5.034965, 58.088578, -35.163170, 1675.821678
        )

        val TOF_SCORING_POLYNOMIAL      : Result<Polynomials> = Polynomials.of(degree = 3,
            -0.006887, 0.058667, -0.109770, 0.986263
        )

        val ASSIST_POLYNOMIAL           : Result<Polynomials> = Polynomials.of(degree = 3,
            -7.631235, 137.810114, -594.245958, 2571.936791
        )

        val TOF_ASSIST_POLYNOMIAL       : Result<Polynomials> = Polynomials.of(degree = 3,
            -0.003982, 0.084914, -0.454582, 2.013256
        )
    }

    /**
     * Contains initial configuration for the Flywheel motors assuming Phoenix API.
     * Configurations meant to be tunable live, limits, control gains, motion targets and movement presets
     * are all stored in a separate file where they are next to those of all other subsystems (excluding drivetrain).
     * This structure is to have a single file that is regularly consulted by the Software Team, whereas this one
     * remains mostly untouched unless the Design or Electrical Teams change something.
     */
    object PhoenixMotorConfiguration {
        val followerLeftMotorAlignment: MotorAlignmentValue = MotorAlignmentValue.Aligned
        val followerRightMotorAlignment: MotorAlignmentValue = MotorAlignmentValue.Opposed

        private val neutralMode: NeutralModeValue = NeutralModeValue.Coast
        private val motorDirection: InvertedValue = InvertedValue.Clockwise_Positive

        private val supplyCurrentLimit: Current = 35.0.amps
        private val statorCurrentLimit: Current = 50.0.amps

        val initialMotorsConfiguration: TalonFXConfiguration = KrakenMotors.createTalonFXConfiguration(
            Optional.of<MotorOutputConfigs>(
                KrakenMotors.configureMotorOutputs(neutralMode, motorDirection)
            ),
            Optional.of<CurrentLimitsConfigs>(
                KrakenMotors.configureCurrentLimits(supplyCurrentLimit, statorCurrentLimit)
            ),
            Optional.of<Slot0Configs>(SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureAngularMotionMagic(
                    SubsystemsMotionTargets.FLYWHEEL_MOTION_TARGETS,
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
        const val SUBSYSTEM_TAB                         : String = "Flywheel"

        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"
        const val FOLLOWER_LEFT_MOTOR_INPUTS_TAB        : String = "${SUBSYSTEM_TAB}/Follower Left"
        const val FOLLOWER_RIGHT_MOTOR_ONE_INPUTS_TAB   : String = "${SUBSYSTEM_TAB}/Follower Right One"
        const val FOLLOWER_RIGHT_MOTOR_TWO_INPUTS_TAB   : String = "${SUBSYSTEM_TAB}/Follower Right Two"

        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"

        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.FLYWHEEL_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
        const val FOLLOWER_MOTOR_CONNECTION_ALERT_TAB   : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.FLYWHEEL_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.FOLLOWER_RIGHT_ONE_ID}"
    }
}
