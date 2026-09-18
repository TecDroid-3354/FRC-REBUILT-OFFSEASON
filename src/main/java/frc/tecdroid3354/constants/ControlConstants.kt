package frc.tecdroid3354.constants

import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.AngularVelocityUnit
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.tecdroid3354.generated.SwerveTunerConstants
import frc.tecdroid3354.subsystems.Hood.HoodConstants
import frc.tecdroid3354.subsystems.Flywheel.FlywheelConstants
import frc.tecdroid3354.subsystems.Hopper.HopperConstants
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployConstants
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersConstants
import frc.tecdroid3354.subsystems.Tower.TowerConstants
import frc.tecdroid3354.utils.controlProfiles.AngularMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LinearMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber
import frc.tecdroid3354.utils.controlProfiles.TunableControlGains
import frc.tecdroid3354.utils.controlProfiles.ControlGains
import frc.tecdroid3354.utils.controlProfiles.Polynomials
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.degreesPerSecond
import frc.tecdroid3354.utils.devices.OpPositionControlRequests
import frc.tecdroid3354.utils.devices.OpPositionControlRequests.*
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests.*
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.rotationsPerMinute
import frc.tecdroid3354.utils.safety.MeasureLimits
import frc.tecdroid3354.utils.seconds

/** Meant for the [edu.wpi.first.wpilibj2.command.button.CommandXboxController] of the driver */
object DriveMultipliers {
    const val CONTROLLER_PRIMARY_X_MULTIPLIER       : Double = 0.8
    const val CONTROLLER_PRIMARY_Y_MULTIPLIER       : Double = 0.8
    const val CONTROLLER_PRIMARY_THETA_MULTIPLIER   : Double = 0.6 // Only in case of continuous rotation

    // Used for Shoot on the Move
    const val CONTROLLER_SOTM_LIMIT_MULTIPLIER      : Double = 0.45
}

/**
 * Stores an Enum value with the Control Request Type used for each subsystem.
 * The corresponding value must be called inside the subsystem's hardware layer when commanding the motor(s) to move.
 */
object SubsystemsControlRequests {
    val HOOD_CONTROL_TYPE               : OpPositionControlRequests = POSITION_TORQUE
    val FLYWHEEL_CONTROL_TYPE           : OpVelocityControlRequests = VELOCITY_TORQUE
    val TOWER_CONTROL_TYPE              : OpVelocityControlRequests = VELOCITY_TORQUE
    val HOPPER_CONTROL_TYPE             : OpVelocityControlRequests = VELOCITY_TORQUE
    val INTAKE_DEPLOY_CONTROL_TYPE      : OpPositionControlRequests = POSITION_DYNAMIC_TORQUE
    val INTAKE_ROLLERS_CONTROL_TYPE     : OpVelocityControlRequests = VELOCITY_TORQUE
}

/** Stores the tolerance of each subsystem. This also includes tolerances for sequences and simulation. */
object SubsystemTolerances {
    /** Calculated from different setpoints and their time to be reached. AdvantageScope graphs were used.
     * Output is in seconds. Used to vary the time before enabling Hopper and Tower. */
    val FLYWHEEL_AT_TARGET_POLYNOMIAL                   : Result<Polynomials> = Polynomials.of(degree = 3,
        0.0, 0.0, 0.0, 0.0
    )

    val DISTANCE_TIME_FILTER                            : Time = 0.05.seconds
    val VELOCITY_TIME_FILTER                            : Time = 0.06.seconds

    val TIME_BEFORE_SCORE_SHOOTING                      : Time = 1.2.seconds
    val TIME_BEFORE_ASSIST_SHOOTING                     : Time = 1.35.seconds
    val FLYWHEEL_TARGET_TOLERANCE                       : AngularVelocity = 45.0.rotationsPerMinute
    val INTAKE_DEPLOY_TOLERANCE                         : Distance = 0.5.inches
    val INTAKE_TIME_TOLERANCE_BEFORE_CLUSTER_SCORE      : Time = TIME_BEFORE_SCORE_SHOOTING.plus(0.02.seconds)
    val INTAKE_TIME_TOLERANCE_BEFORE_CLUSTER_ASSIST     : Time = TIME_BEFORE_ASSIST_SHOOTING.plus(0.02.seconds)

    val DRIVE_HEADING_TOLERANCE                         : Angle = 2.0.degrees

    val AUTONOMOUS_SCORING_SEQUENCE_TIME_TOLERANCE      : Time = 4.2.seconds

    // Throughput of ~ 18 bps, which is a bit less because of loop cycles.
    val SIMULATION_FUEL_LAUNCH_TIMEOUT_TOLERANCE        : Time = 0.0555.seconds
    const val SIMULATION_HOPPER_FUEL_CAPACITY_TOLERANCE : Int = 56
}

/**
 * Each subsystem set of [frc.tecdroid3354.utils.safety.MeasureLimits]. Naming must be as follows:
 * subsystemName_limits
 */
object SubsystemsMovementLimits {
    //
    // HOOD ONLY
    //
    val HOOD_POSITION_LIMITS: MeasureLimits<AngleUnit> =
        MeasureLimits(0.0.degrees .. 30.0.degrees)

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 4_200.0.rotationsPerMinute)

    //
    // TOWER ONLY
    //
    val TOWER_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 3_850.0.rotationsPerMinute)

    //
    // HOPPER ONLY
    //
    val HOPPER_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 650.0.rotationsPerMinute)

    //
    // ELEVATOR ONLY
    //
    val INTAKE_DEPLOY_DISPLACEMENT_LIMITS: MeasureLimits<DistanceUnit> =
        MeasureLimits(0.0.inches .. 11.8.inches)

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 3_000.0.rotationsPerMinute)
}

/** For all known targets of each subsystem */
object SubsystemsPresetTargets {
    //
    // HOOD ONLY
    //
    val HOOD_HOME_ANGLE: Angle = 0.0.degrees

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_PRESET_RPM: AngularVelocity = 3_200.0.rotationsPerMinute

    //
    // TOWER ONLY
    //
    val TOWER_PRESET_RPM: AngularVelocity = 3_750.0.rotationsPerMinute

    //
    // HOPPER ONLY
    //
    val HOPPER_PRESET_RPM: AngularVelocity = 625.0.rotationsPerMinute

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_HOME_DISPLACEMENT: Distance = 0.0.inches
    val INTAKE_DEPLOY_CLUSTERING_DISPLACEMENT: Distance = 5.45.inches
    val INTAKE_DEPLOY_EXTENDED_DISPLACEMENT: Distance = 11.8.inches

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_PRESET_RPM: AngularVelocity = 2_500.0.rotationsPerMinute
}

/**
 * Each subsystem set of [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]. Naming must be as follows:
 * subsystemName_[kP/kI/kD/kF/manualTargetRPMs/manualTargetAngle/...]
 */
object SubsystemsTunableTargets {
    //
    // HOOD ONLY
    //
    val HOOD_MANUAL_TARGET_DEGREES: LoggedTunableNumber =
        LoggedTunableNumber("${ HoodConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (deg)", 20.0)

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ FlywheelConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 3_800.0)

    //
    // TOWER ONLY
    //
    val TOWER_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ TowerConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 3_600.0)

    //
    // HOPPER ONLY
    //
    val HOPPER_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ HopperConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 575.0)

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_MANUAL_TARGET_INCHES: LoggedTunableNumber =
        LoggedTunableNumber("${ IntakeDeployConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (in)", 11.8)

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${IntakeRollersConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 2_500.0)
}

 /**
 * Each subsystem set of [TunableControlGains]. Note that this are still tunable, yet specifically for [ControlGains],
 * which is why they are here.
 *
 * Naming must be as follows:
 * ***SUBSYSTEM_MOTOR_[[PRIMARY/SECONDARY/TERTIARY]]_GAINS***, where ***[[PRIMARY/SECONDARY/TERTIARY]]***
 * refers to slot 0,1,2 configs in Phoenix.
 *
 * For readability, ensure you specify the argument name before the coefficients.
 */
object SubsystemsControlGains {
     //
     // Hood ONLY
     //
     val HOOD_MOTOR_PRIMARY_GAINS      : TunableControlGains = TunableControlGains(HoodConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 36.5, kI = 0.0, kD = 3.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.35) // TUNED

    //
    // FLYWHEEL ONLY
    //
     val FLYWHEEL_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(FlywheelConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 8.25, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.3, kA = 0.0, kG = 0.0) // TODO() = TRY WITH FUELS

     //
     // TOWER ONLY
     //
     val TOWER_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(TowerConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // TUNED

     //
     // HOPPER ONLY
     //
     val HOPPER_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(HopperConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 7.35, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // TUNED

     //
     // INTAKE DEPLOY ONLY
     //
     val INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(IntakeDeployConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 14.75, kI = 0.0, kD = 0.4, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // SEMI - TUNED

     //
     // INTAKE ROLLERS ONLY
     //
     val INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(IntakeRollersConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 25.0, kI = 0.0, kD = 0.0, kS = 0.65, kV = 1.0, kA = 0.0, kG = 0.0) // TUNED

     //
     // DRIVE ONLY
     //
     val CHASSIS_AUTONOMOUS_CONTROLLER    : PPHolonomicDriveController = PPHolonomicDriveController(
         PIDConstants(13.5, 0.0, 1.75),   // Translational PID
         PIDConstants(18.25, 0.0, 2.25)    // Rotational PID
     ) // Note that this is not live-tunable because PathPlanner creates an immutable PID object with the first configuration.

     // Note that these values cannot be accurately tuned in simulation, unlike the autonomous controller.
     val DRIVE_MOTOR_PRIMARY_GAINS        : TunableControlGains = TunableControlGains(SwerveTunerConstants.SUBSYSTEM_DRIVE_PRIMARY_GAINS,
         kP = 1.45, kI = 0.0, kD = 0.05, kS = 0.1, kV = 0.7, kA = 0.0, kG = 0.0)    // TUNED FOR VOLTAGE
     val STEER_MOTOR_PRIMARY_GAINS        : TunableControlGains = TunableControlGains(SwerveTunerConstants.SUBSYSTEM_STEER_PRIMARY_GAINS,
         kP = 32.5, kI = 0.0, kD = 0.35, kS = 0.2, kV = 0.0, kA = 0.0, kG = 0.0)   // TUNED FOR VOLTAGE
}

/**
 * Each subsystem set of [frc.tecdroid3354.utils.controlProfiles.MotionTargets]. Naming must be as follows:
 * SUBSYSTEM_MOTION_TARGETS.
 *
 * For dynamic subsystems that require more than 1 set of motion targets, specify if the variable contains the
 * primary, secondary, or tertiary motion targets.
 */
object SubsystemsMotionTargets {
    //
    // HOOD ONLY
    //
    val HOOD_PRIMARY_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            100.0.degreesPerSecond,
            0.05.seconds,
            0.0.seconds
        )

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_200.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // TOWER ONLY
    //
    val TOWER_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            3_850.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // HOPPER ONLY
    //
    val HOPPER_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            650.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_PRIMARY_MOTION_TARGETS: LinearMotionTargets = // Standard motion
        LinearMotionTargets(
            0.75.metersPerSecond,
            0.075.seconds,
            Seconds.zero(),
        )

    val INTAKE_DEPLOY_SECONDARY_MOTION_TARGETS: LinearMotionTargets = // For cluster motion
        LinearMotionTargets( // Same as Primary for testing, commented values would be for real manually-controlled motion
            0.75.metersPerSecond,
            0.15.seconds,
            0.02.seconds,
        )

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            3_000.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )
}
