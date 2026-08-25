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
import frc.tecdroid3354.subsystems.angularPosition.JointConstants
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelConstants
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorConstants
import frc.tecdroid3354.utils.controlProfiles.AngularMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LinearMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber
import frc.tecdroid3354.utils.controlProfiles.TunableControlGains
import frc.tecdroid3354.utils.controlProfiles.ControlGains
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.devices.OpPositionControlRequests
import frc.tecdroid3354.utils.devices.OpPositionControlRequests.POSITION_DYNAMIC_TORQUE
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests
import frc.tecdroid3354.utils.devices.OpVelocityControlRequests.VELOCITY_TORQUE
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
    val FLYWHEEL_CONTROL_TYPE   : OpVelocityControlRequests = VELOCITY_TORQUE
    val JOINT_CONTROL_TYPE      : OpPositionControlRequests = POSITION_DYNAMIC_TORQUE
    val ELEVATOR_CONTROL_TYPE   : OpPositionControlRequests = POSITION_DYNAMIC_TORQUE
}

/**
 * Each subsystem set of [frc.tecdroid3354.utils.safety.MeasureLimits]. Naming must be as follows:
 * subsystemName_limits
 */
object SubsystemsMovementLimits {
    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_VELOCITY_LIMITS: MeasureLimits<AngularVelocityUnit> =
        MeasureLimits(0.0.rotationsPerMinute .. 4_200.0.rotationsPerMinute)

    //
    // JOINT ONLY
    //
    val JOINT_POSITION_LIMITS: MeasureLimits<AngleUnit> =
        MeasureLimits(20.0.degrees .. 100.0.degrees)

    //
    // ELEVATOR ONLY
    //
    val ELEVATOR_DISPLACEMENT_LIMITS: MeasureLimits<DistanceUnit> =
        MeasureLimits(0.0.inches .. 52.0.inches)
}

/** For all known targets of each subsystem */
object SubsystemsPresetTargets {
    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_PRESET_RPM: AngularVelocity = 3_200.0.rotationsPerMinute

    //
    // JOINT ONLY
    //
    val JOINT_IDLE_ANGLE: Angle = 45.0.degrees
    val JOINT_HOME_ANGLE: Angle = 90.0.degrees

    //
    // ELEVATOR ONLY
    //
    val ELEVATOR_IDLE_DISPLACEMENT: Distance = 10.0.inches
    val ELEVATOR_HOME_DISPLACEMENT: Distance = 0.0.inches
}

/**
 * Each subsystem set of [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]. Naming must be as follows:
 * subsystemName_[kP/kI/kD/kF/manualTargetRPMs/manualTargetAngle/...]
 */
object SubsystemsTunableTargets {
    //
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MANUAL_RPM: LoggedTunableNumber =
        LoggedTunableNumber("${ FlywheelConstants.Telemetry.SUBSYSTEM_TAB }/Manual RPMs", 1_800.0)

    //
    // JOINT ONLY
    //
    val JOINT_MANUAL_TARGET_DEGREES: LoggedTunableNumber =
        LoggedTunableNumber("${ JointConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (deg)", 45.0)

    //
    // ELEVATOR ONLY
    //
    val ELEVATOR_MANUAL_TARGET_INCHES: LoggedTunableNumber =
        LoggedTunableNumber("${ ElevatorConstants.Telemetry.SUBSYSTEM_TAB }/Manual Target (in)", 8.0)
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
    // FLYWHEEL ONLY
    //
     val FLYWHEEL_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(FlywheelConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // "Tuned" in SIMULATION -> Torque Request (Probably wrong MOI)
     val FLYWHEEL_MOTOR_SECONDARY_GAINS : TunableControlGains = TunableControlGains(FlywheelConstants.Telemetry.SUBSYSTEM_SECONDARY_GAINS,
         kP = 0.5, kI = 0.0, kD = 0.0, kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0) // Not Tuned

     //
     // ELEVATOR ONLY
     //
     val ELEVATOR_MOTOR_PRIMARY_GAINS   : TunableControlGains = TunableControlGains(ElevatorConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
        kP = 450.0, kI = 0.0, kD = 10.0, kS = 0.0, kV = 0.0, kA = 0.75, kG = 6.52) // Tuned in SIMULATION -> Torque Request.

     //
     // JOINT ONLY
     //
     val JOINT_MOTOR_PRIMARY_GAINS      : TunableControlGains = TunableControlGains(JointConstants.Telemetry.SUBSYSTEM_PRIMARY_GAINS,
         kP = 87.5, kI = 0.0, kD = 12.5, kS = 61.0, kV = 0.0, kA = 0.0, kG = 189.0) // Tuned in SIMULATION -> Torque Request

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
    // FLYWHEEL ONLY
    //
    val FLYWHEEL_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets(
            4_500.0.rotationsPerMinute,
            0.1.seconds,
            Seconds.zero(),
        )

    //
    // JOINT ONLY
    //
    val JOINT_PRIMARY_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets( // From 0 radians to PI/2 radians ~ 1.2 seconds (1 for PI/2, 1 from acc, 1 from jerk)
            Math.PI.div(2).radiansPerSecond,
            0.1.seconds,
            0.1.seconds
        )

    val JOINT_SECONDARY_MOTION_TARGETS: AngularMotionTargets =
        AngularMotionTargets( // Half the cruise velocity of Primary Targets
            Math.PI.div(2).radiansPerSecond, // Same as Primary for testing in simulation
            0.1.seconds,
            0.1.seconds
        )

    //
    // ELEVATOR ONLY
    //
    val ELEVATOR_PRIMARY_MOTION_TARGETS: LinearMotionTargets = // Standard motion
        LinearMotionTargets(
            1.2.metersPerSecond,
            0.1.seconds,
            0.1.seconds,
        )

    val ELEVATOR_SECONDARY_MOTION_TARGETS: LinearMotionTargets = // For manual motion
        LinearMotionTargets( // Same as Primary for testing, commented values would be for real manually-controlled motion
            1.2.metersPerSecond, // 0.8
            0.1.seconds, // 0.8
            0.1.seconds, // 0.5
        )
}
