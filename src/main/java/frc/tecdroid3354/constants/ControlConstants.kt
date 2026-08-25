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
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.devices.OpPositionControlRequests
import frc.tecdroid3354.utils.devices.OpPositionControlRequests.*
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests.*
import frc.tecdroid3354.utils.inches
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.radiansPerSecond
import frc.tecdroid3354.utils.rotationsPerMinute
import frc.tecdroid3354.utils.safety.MeasureLimits
import frc.tecdroid3354.utils.seconds

/** Meant for the [edu.wpi.first.wpilibj2.command.button.CommandXboxController] of the driver */
object DriveMultipliers {
    const val CONTROLLER_PRIMARY_X_MULTIPLIER       : Double = 0.8
    const val CONTROLLER_PRIMARY_Y_MULTIPLIER       : Double = 0.8
    const val CONTROLLER_PRIMARY_THETA_MULTIPLIER   : Double = 0.6 // Only in case of continuous rotation
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
        MeasureLimits(0.0.rotationsPerMinute .. 4_200.0.rotationsPerMinute)

    //
    // HOPPER ONLY
    //
    val HOPPER_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 4_200.0.rotationsPerMinute)

    //
    // ELEVATOR ONLY
    //
    val INTAKE_DEPLOY_DISPLACEMENT_LIMITS: MeasureLimits<DistanceUnit> =
        MeasureLimits(0.0.inches .. 52.0.inches)

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 4_200.0.rotationsPerMinute)
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
    val TOWER_PRESET_RPM: AngularVelocity = 3_200.0.rotationsPerMinute

    //
    // HOPPER ONLY
    //
    val HOPPER_PRESET_RPM: AngularVelocity = 3_200.0.rotationsPerMinute

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_HOME_DISPLACEMENT: Distance = 0.0.inches
    val INTAKE_DEPLOY_IDLE_DISPLACEMENT: Distance = 10.0.inches

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_PRESET_RPM: AngularVelocity = 3_200.0.rotationsPerMinute
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
        LoggedTunableNumber("${ HoodConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (deg)", 0.0)

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ FlywheelConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 1_800.0)

    //
    // TOWER ONLY
    //
    val TOWER_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ TowerConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 1_800.0)

    //
    // HOPPER ONLY
    //
    val HOPPER_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ HopperConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 1_800.0)

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_MANUAL_TARGET_INCHES: LoggedTunableNumber =
        LoggedTunableNumber("${ IntakeDeployConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (in)", 8.0)

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${IntakeRollersConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 1_800.0)
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
         kP = 87.5, kI = 0.0, kD = 12.5, kS = 61.0, kV = 0.0, kA = 0.0, kG = 189.0) // Tuned in SIMULATION -> Torque Request

    //
    // FLYWHEEL ONLY
    //
     val FLYWHEEL_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(FlywheelConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // "Tuned" in SIMULATION -> Torque Request (Probably wrong MOI)

     //
     // TOWER ONLY
     //
     val TOWER_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(TowerConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // "Tuned" in SIMULATION -> Torque Request (Probably wrong MOI)

     //
     // HOPPER ONLY
     //
     val HOPPER_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(HopperConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // "Tuned" in SIMULATION -> Torque Request (Probably wrong MOI)

     //
     // INTAKE DEPLOY ONLY
     //
     val INTAKE_DEPLOY_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(IntakeDeployConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 450.0, kI = 0.0, kD = 10.0, kS = 0.0, kV = 0.0, kA = 0.75, kG = 6.52) // Tuned in SIMULATION -> Torque Request.

     //
     // INTAKE ROLLERS ONLY
     //
     val INTAKE_ROLLERS_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(IntakeRollersConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // "Tuned" in SIMULATION -> Torque Request (Probably wrong MOI)

     //
     // DRIVE ONLY
     //
     val CHASSIS_AUTONOMOUS_CONTROLLER    : PPHolonomicDriveController = PPHolonomicDriveController(
         PIDConstants(6.0, 0.0, 0.0),   // Translational PID
         PIDConstants(10.0, 0.0, 0.0)    // Rotational PID
     ) // Note that this is not live-tunable because PathPlanner creates an immutable PID object with the first configuration.

     // Note that these values cannot be accurately tuned in simulation, unlike the autonomous controller.
     val DRIVE_MOTOR_PRIMARY_GAINS        : TunableControlGains = TunableControlGains(SwerveTunerConstants.SUBSYSTEM_DRIVE_PRIMARY_GAINS,
         kP = 0.8, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.124, kA = 0.0, kG = 0.0)    // TODO() = Tune for Torque in REAL robot
     val STEER_MOTOR_PRIMARY_GAINS        : TunableControlGains = TunableControlGains(SwerveTunerConstants.SUBSYSTEM_STEER_PRIMARY_GAINS,
         kP = 100.0, kI = 0.0, kD = 0.5, kS = 0.1, kV = 2.49, kA = 0.0, kG = 0.0)   // TODO() = Tune for Torque in REAL robot
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
        AngularMotionTargets( // From 0 radians to PI/2 radians ~ 1.2 seconds (1 for PI/2, 1 from acc, 1 from jerk)
            Math.PI.div(2).radiansPerSecond,
            0.1.seconds,
            0.1.seconds
        )

    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_500.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // TOWER ONLY
    //
    val TOWER_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_500.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // HOPPER ONLY
    //
    val HOPPER_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_500.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // INTAKE DEPLOY ONLY
    //
    val INTAKE_DEPLOY_PRIMARY_MOTION_TARGETS: LinearMotionTargets = // Standard motion
        LinearMotionTargets(
            1.2.metersPerSecond,
            0.1.seconds,
            0.1.seconds,
        )

    val INTAKE_DEPLOY_SECONDARY_MOTION_TARGETS: LinearMotionTargets = // For manual motion
        LinearMotionTargets( // Same as Primary for testing, commented values would be for real manually-controlled motion
            1.2.metersPerSecond, // 0.8
            0.1.seconds, // 0.8
            0.1.seconds, // 0.5
        )

    //
    // INTAKE ROLLERS ONLY
    //
    val INTAKE_ROLLERS_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_500.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )
}
