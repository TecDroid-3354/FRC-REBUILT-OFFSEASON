package frc.tecdroid3354.constants

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.degreesCelsius
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.kilogramSquareMeters
import frc.tecdroid3354.utils.kilograms
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.milliseconds
import java.util.function.BooleanSupplier

enum class RobotMode {
    REAL,
    SIM,
    REPLAY
}

object RobotConstants {
    val ROBOT_MODE                      : RobotMode = if (RobotBase.isReal()) RobotMode.REAL else RobotMode.SIM
    val LOOP_TIME                       : Time = 20.0.milliseconds
    val IS_RED_ALLIANCE                 : BooleanSupplier =
        { DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red }

    const val TUNING_MODE               : Boolean = true // Enables live tuning through Elastic.
    const val MAIN_CONTROLLER_PORT      : Int = 0
}

/** Contains robot-level values that affect movement calculations, such as mass, MOI and wheel cof */
object RobotPhysics {
    val RobotMass       : Mass              = 52.896.kilograms
    // Measured from Tutankabot's CAD as of 02/08/2026 -> Selected all Main Layout and picked Lzz MOI from Mass & Properties
    // This gives around ~ 5.396 kg * m^2
    val RobotMOI        : MomentOfInertia   = (18_440.2857 * SimConstants.FREEDOM_UNITS_TO_METRIC_MOI).kilogramSquareMeters

    // This is an estimate that assumes the robot as a solid rectangular plate of uniformly-distributed mass.
    // Since the robot tends to have non-uniformly distributed mass, this tends to be an underestimate.
    // In this case, this gives around ~ 3.8 kg * m^2
//    val RobotMOI        : MomentOfInertia   = ((1/12) * (RobotMass.kilograms) *
//            (RobotLength.meters.pow(2) + RobotWidth.meters.pow(2))).kilogramSquareMeters

    const val WHEEL_COF : Double            = 1.2
}

/**
 * Contains all fixed dimensions of the robot that may be relevant. This includes measures relevant for simulation.
 */
object RobotDimensions {
    val BUMPERS_HEIGHT                  : Distance = 5.0.inches
    val BUMPERS_DEPTH                   : Distance = 2.5.inches

    val ROBOT_LENGTH                    : Distance = 26.5.inches.plus(BUMPERS_DEPTH)
    val ROBOT_WIDTH                     : Distance = 26.5.inches.plus(BUMPERS_DEPTH)
    val ROBOT_HEIGHT                    : Distance = 0.6.meters

    val INTAKE_LENGTH                   : Distance = 7.5.inches // Examples only, not included in template
    val INTAKE_WIDTH                    : Distance = 26.5.inches

    val JOINT_WIDTH                     : Distance = 6.5.inches
    val JOINT_FORWARD_OFFSET            : Distance = (-0.19685).meters
    val JOINT_UPWARD_OFFSET             : Distance = 0.1048.meters

    val ELEVATOR_MINIMUM_LENGTH         : Distance = 0.8.meters     // Note that "length" differs from "displacement"
    val ELEVATOR_MAXIMUM_LENGTH         : Distance = 2.32.meters    // Minimum + Maximum displacement
}

/**
 * Contains all [edu.wpi.first.math.geometry.Translation2d] and [edu.wpi.first.math.geometry.Translation3d]
 * relevant to the robot. This includes simulation-relevant transformations.
 *
 * i.e., Robot's transformation from center to a subsystem.
 */
object RobotTransformations {
    val ROBOT_TO_SHOOTER = Transform3d(0.0.meters, (-0.15).meters, 0.5.meters,
        Rotation3d(0.0.degrees, 0.0.degrees, 180.0.degrees))
    val ROBOT_TO_INTAKE = Translation2d(0.0.meters, 0.25.meters)
}

/**
 * Contains the name of each canbus present in the robot.
 * While every subsystem stores its own variable for this, said variable must be assigned to one of these values
 * to prevent any mismatch between IDs.
 */
object CanBuses {
    const val RIO_CANBUS         : String = "rio"
    const val CANIVORE_CANBUS    : String = "canivore"
}

/** Stores values that are only used for simulation */
object SimConstants {
    val NEUTRAL_MOTOR_TEMPERATURE: Temperature = 45.0.degreesCelsius

    // Based off Falcon 500, which should be fairly similar to Kraken x60 MOI
    val ESTIMATED_MOTOR_MOI                     : MomentOfInertia = (3.6e-5).kilogramSquareMeters
    const val FREEDOM_UNITS_TO_METRIC_MOI       : Double = 0.00029263965
}

/**
 * For robot-wise telemetry tabs.
 */
object RobotTelemetry {
    const val CONNECTION_ALERTS_TAB             : String = "Connection Alerts" // Usage: thisTab/Canbus/subsystemName + motor id
    const val ROBOT_MODE_TAB                    : String = "RobotMode"
    const val SUBSYSTEM_VISUALIZATION_2D_TAB    : String = "Subsystems Mechanism2d"
    const val SUBSYSTEM_VISUALIZATION_3D_TAB    : String = "Subsystems Pose3d"
}

/**
 * For all 2D / 3D robot visualization constants, which will then assemble all subsystems inside [frc.tecdroid3354.RobotVisualizer]
 */
object RobotVisualization {
    const val MECHANISMS_ORIGIN_2D_NAME                     : String = "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB} Origin"
    const val MECHANISMS_ARM_GUIDING_RAIL_2D_NAME           : String = "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB} Guiding Rail"
    const val MECHANISMS_ARM_DISPLACEMENT_LIGAMENT_2D_NAME  : String = "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB} Displacement Ligament"
    const val MECHANISMS_ARM_END_EFFECTOR_2D_NAME           : String = "${RobotTelemetry.SUBSYSTEM_VISUALIZATION_2D_TAB} Carriage"

    val CANVAS_WIDTH                                        : Distance = 5.0.meters
    val CANVAS_HEIGHT                                       : Distance = 3.0.meters
    val CANVAS_COLOR                                        : Color8Bit = Color8Bit(Color.kDarkGray)

    val ARM_GUIDING_RAIL_WIDTH                              : Double = 6.0 // Who knows the unit of this
    val GUIDING_RAIL_COLOR                                  : Color8Bit = Color8Bit(Color.kDarkViolet)

    val ARM_DISPLACEMENT_LIGAMENT_WIDTH                     : Double = 4.0
    val ARM_DISPLACEMENT_LIGAMENT_INITIAL_ANGLE             : Angle = 0.0.degrees // Relative to guiding rail
    val ARM_DISPLACEMENT_LIGAMENT_COLOR                     : Color8Bit = Color8Bit(Color.kBlack)

    val ARM_END_EFFECTOR_WIDTH                              : Double = 6.0
    val ARM_END_EFFECTOR_HEIGHT                             : Distance = 0.05.meters
    val ARM_END_EFFECTOR_RELATIVE_ANGLE                     : Angle = 0.0.degrees // Relative to displacement ligament
    val ARM_END_EFFECTOR_COLOR                              : Color8Bit = Color8Bit(Color.kGold)
}