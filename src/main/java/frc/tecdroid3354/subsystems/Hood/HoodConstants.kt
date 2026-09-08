package frc.tecdroid3354.subsystems.Hood

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

object HoodConstants {
    /**
     * Contains the ID Of any hardware related to the subsystem and the CANBUS it is on
     */
    object Identification {
        const val HOOD_CANBUS_NAME: String = CanBuses.RIO_CANBUS
        const val LEAD_MOTOR_ID: Int = 51
    }

    /**
     * Only for gear ratio ([Reduction]). In the case of linear subsystems, the sprocket also goes here.
     */
    object Mechanical {
        val REDUCTION: Reduction = Reduction((142.0 / 13.0) * (60.0 / 9.0) * (37.0 / 22.0))

        const val NUMBER_OF_MOTORS: Int = 1

        // From OnShape, as of 26/08/2026
        private val MECHANISM_INERTIA: MomentOfInertia = (154.498876.times(SimConstants.FREEDOM_UNITS_TO_METRIC_MOI)).kilogramSquareMeters

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
     * Only in the scenario you use a polynomial to calculate the target hood angle.
     * In this example we assume two polynomials, one for scoring and another for assist (based off 2026 REBUILT)
     *
     * All polynomials were calculated from TecDroid's Trajectory Simulator.
     */
    object PolynomialCoefficients {
        const val SCORING_X3_COEFF: Double = -0.257860
        const val SCORING_X2_COEFF: Double = 0.823323
        const val SCORING_X1_COEFF: Double = 8.641864
        const val SCORING_X0_COEFF: Double = 0.238287

        const val ASSIST_X3_COEFF: Double = 0.632129
        const val ASSIST_X2_COEFF: Double = -8.336842
        const val ASSIST_X1_COEFF: Double = 35.949862
        const val ASSIST_X0_COEFF: Double = -20.551522
    }

    /**
     * Contains initial configuration for the subsystem motors assuming Phoenix API.
     * Configurations meant to be tunable live, limits, control gains, motion targets and movement presets
     * are all stored in a separate file where they are next to those of all other subsystems (excluding drivetrain).
     * This structure is to have a single file that is regularly consulted by the Software Team, whereas this one
     * remains mostly untouched unless the Design or Electrical Teams change something.
     */
    object PhoenixMotorConfiguration {
        private val neutralMode: NeutralModeValue = NeutralModeValue.Brake
        private val motorDirection: InvertedValue = InvertedValue.Clockwise_Positive

        private val supplyCurrentLimit: Current = 30.0.amps
        private val statorCurrentLimit: Current = 80.0.amps

        val initialMotorsConfiguration: TalonFXConfiguration = KrakenMotors.createTalonFXConfiguration(
            Optional.of<MotorOutputConfigs>(
                KrakenMotors.configureMotorOutputs(neutralMode, motorDirection)
            ),
            Optional.of<CurrentLimitsConfigs>(
                KrakenMotors.configureCurrentLimits(supplyCurrentLimit, statorCurrentLimit)
            ),
            Optional.of<Slot0Configs>(SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.HOOD_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureAngularMotionMagic(
                    SubsystemsMotionTargets.HOOD_PRIMARY_MOTION_TARGETS,
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
        const val SUBSYSTEM_TAB                         : String = "Hood"
        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"

        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"

        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.HOOD_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
    }
}
