package frc.tecdroid3354.subsystems.Hopper

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import frc.tecdroid3354.constants.*
import frc.tecdroid3354.utils.Sprocket
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.devices.KrakenMotors
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.kilograms
import frc.tecdroid3354.utils.mechanical.Reduction
import frc.tecdroid3354.utils.pounds
import java.util.*
import kotlin.math.pow

object HopperConstants {
    /**
     * Contains the ID Of any hardware related to the subsystem and the CANBUS it is on
     */
    object Identification {
        const val HOPPER_CANBUS_NAME: String = CanBuses.RIO_CANBUS
        const val LEAD_MOTOR_ID: Int = 30
    }

    /**
     * Only for gear ratio ([Reduction]) and MOI (for simulation)
     * In the case of linear subsystems, the sprocket also goes here.
     */
    object Mechanical {
        val REDUCTION: Reduction = Reduction(7.35)

        const val NUMBER_OF_MOTORS: Int = 1

        // From OnShape as of 26/08/2026. WHOLE Belts -> 1120.670437,   ONLY pullies -> 0.206119 * 2 = 0.412238
        private val MECHANISM_INERTIA: MomentOfInertia = (1120.670437.times(SimConstants.FREEDOM_UNITS_TO_METRIC_MOI)).kilogramSquareMeters

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

        val SPROCKET: Sprocket = Sprocket.fromRadius(0.783.inches)
        val MIN_MASS: Mass = 3.778.kilograms
        val MAX_MASS: Mass = MIN_MASS.plus(
            0.2.kilograms.times(70.0)
        )
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
        private val motorDirection: InvertedValue = InvertedValue.CounterClockwise_Positive

        private val supplyCurrentLimit: Current = 40.0.amps
        private val statorCurrentLimit: Current = 80.0.amps

        val initialMotorsConfiguration: TalonFXConfiguration = KrakenMotors.createTalonFXConfiguration(
            Optional.of<MotorOutputConfigs>(
                KrakenMotors.configureMotorOutputs(neutralMode, motorDirection)
            ),
            Optional.of<CurrentLimitsConfigs>(
                KrakenMotors.configureCurrentLimits(supplyCurrentLimit, statorCurrentLimit)
            ),
            Optional.of<Slot0Configs>(SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()),
            Optional.of<Slot1Configs>(SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()),
            Optional.of<Slot2Configs>(SubsystemsControlGains.HOPPER_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()),
            Optional.of<MotionMagicConfigs>(
                KrakenMotors.configureAngularMotionMagic(
                    SubsystemsMotionTargets.HOPPER_MOTION_TARGETS,
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
        const val SUBSYSTEM_TAB                         : String = "Hopper"
        const val LEAD_MOTOR_INPUTS_TAB                 : String = "${SUBSYSTEM_TAB}/Lead Motor"
        const val SUBSYSTEM_PRIMARY_GAINS               : String = "$SUBSYSTEM_TAB Primary Gains"

        const val LEAD_MOTOR_CONNECTION_ALERT_TAB       : String =
            "${RobotTelemetry.CONNECTION_ALERTS_TAB}/${Identification.HOPPER_CANBUS_NAME}" +
                    "/${SUBSYSTEM_TAB} Motor id=${Identification.LEAD_MOTOR_ID}"
    }
}
