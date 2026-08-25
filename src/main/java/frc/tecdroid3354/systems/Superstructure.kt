package frc.tecdroid3354.systems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.tecdroid3354.commands.DriveCommands
import frc.tecdroid3354.constants.DriveMultipliers
import frc.tecdroid3354.constants.FieldConstants.TargetTranslations
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotConstants.IS_RED_ALLIANCE
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.constants.RobotTransformations
import frc.tecdroid3354.subsystems.angularPosition.JointSubsystem
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelSubsystem
import frc.tecdroid3354.subsystems.drive.Drive
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorSubsystem
import frc.tecdroid3354.subsystems.vision.Vision
import frc.tecdroid3354.utils.InstantCommand
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.toRotation2d
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import java.util.Optional
import java.util.function.Supplier

/** Inside this class, construct all control methods at robot-level (and any relevant auxiliary methods).
 * When using States, this file is responsible to control the logic flow */
class Superstructure(private val controller: CommandPS5Controller,
                     private val simDrive: SwerveDriveSimulation, private val drive: Drive,
                     private val joint: JointSubsystem, private val elevator: ElevatorSubsystem,
                     private val flywheel: FlywheelSubsystem, private val vision: Vision): SubsystemBase("Superstructure") {

    private val fieldRelativeSpeeds: Supplier<ChassisSpeeds> = { drive.fieldRelativeChassisSpeeds }
    private var radialVelocityToHub: LinearVelocity = 0.0.metersPerSecond
    private var tangentialVelocityToHub: LinearVelocity = 0.0.metersPerSecond

    override fun periodic() {
        // Merely assigned to log them
        radialVelocityToHub = DriveCommands.getRobotRadialVelocity(
            fieldRelativeSpeeds.get(), drive.pose,
            if (IS_RED_ALLIANCE.asBoolean) TargetTranslations.RED_HUB else TargetTranslations.BLUE_HUB
        )

        tangentialVelocityToHub = DriveCommands.getRobotTangentialVelocity(
            fieldRelativeSpeeds.get(), drive.pose,
            if (IS_RED_ALLIANCE.asBoolean) TargetTranslations.RED_HUB else TargetTranslations.BLUE_HUB
        )
    }

    // --------------- ----- -------- --------------- //
    // --------------- DRIVE COMMANDS --------------- //
    // --------------- ----- -------- --------------- //

    /** Keeps the reported translation of the odometry, but sets the heading to [headingOffset], or 0 if empty */
    fun resetOdometryHeading(headingOffset: Optional<Rotation2d>): Command {
        if (RobotConstants.ROBOT_MODE == RobotMode.SIM) {
            return InstantCommand( {drive.resetOdometry(
                Pose2d(simDrive.simulatedDriveTrainPose.translation, headingOffset.orElse(Rotation2d())))}
            )
        }
        return InstantCommand({ drive.resetOdometry(Pose2d(drive.pose.translation, headingOffset.orElse(Rotation2d()))) })
    }

    /** Overrides the current odometry to [pose] */
    fun resetOdometryPose(pose: Pose2d): Command {
        return InstantCommand({ drive.resetOdometry(pose) })
    }

    /** Gives full control of translation and rotation to the driver, with field-oriented rotation */
    fun setDriveDefaultCommand() {
        drive.defaultCommand = setDriveTeleopCommand()
    }

    /** After calling this method, the [Drive] will not listen to the driver's controller */
    fun removeDriveDefaultCommand() {
        drive.removeDefaultCommand()
    }

    /** Same as default command, gives the driver full control with field-relative rotation */
    fun setDriveTeleopCommand(): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { MathUtil.applyDeadband(-controller.leftY, 0.05) * DriveMultipliers.CONTROLLER_PRIMARY_Y_MULTIPLIER },
            { MathUtil.applyDeadband(-controller.leftX, 0.05) * DriveMultipliers.CONTROLLER_PRIMARY_X_MULTIPLIER },
            { DriveCommands.getAngleFromJoystick(controller.rightX, controller.rightY).toRotation2d() },
        )
    }

    /** Locks the [drive] rotation to always point at target with the shooter of 2026 Tutankabot */
    fun setDriveTargetingCommand(fieldToTarget: Supplier<Translation2d>): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { MathUtil.applyDeadband(-controller.leftY, 0.05) * DriveMultipliers.CONTROLLER_PRIMARY_Y_MULTIPLIER },
            { MathUtil.applyDeadband(-controller.leftX, 0.05) * DriveMultipliers.CONTROLLER_PRIMARY_X_MULTIPLIER },
            { DriveCommands.getAngleFromRobotToTarget(
                drive.pose, fieldToTarget.get(),
                Optional.of(RobotTransformations.ROBOT_TO_SHOOTER.rotation.measureZ.toRotation2d()),
                true) },
        )
    }

    // --------------- --- -------- --------------- //
    // --------------- ARM COMMANDS --------------- //
    // --------------- --- -------- --------------- //

    /** Commands the preset home positions of the [elevator] and [joint], in that order. Safety delay for joint not considered */
    fun homeArm(): Command {
        return SequentialCommandGroup(
            homeElevator(),
            homeJoint(),
        )
    }

    /** Commands the preset idle positions of the [elevator] and [joint], in that order. Safety delay for joint not considered */
    fun idleArm(): Command {
        return SequentialCommandGroup(
            idleElevator(),
            idleJoint(),
        )
    }

    /** Commands the manual positions of the [elevator] and [joint], in that order. Safety delay for joint not considered */
    fun setArmManualControl(): Command {
        return SequentialCommandGroup(
            setElevatorManualControl(),
            setJointManualControl(),
        )
    }

    // --------------- ----- -------- --------------- //
    // --------------- JOINT COMMANDS --------------- //
    // --------------- ----- -------- --------------- //

    /** Sets the joint to the preset home displacement */
    fun homeJoint(): Command {
        return joint.setJointHomePosition().InstantCommand(joint)
    }

    /** Sets the joint to the preset idle displacement */
    fun idleJoint(): Command {
        return joint.setJointIdlePosition().InstantCommand(joint)
    }

    /** Sets the joint to the manual target displacement */
    fun setJointManualControl(): Command {
        return joint.setJointManualPosition().InstantCommand(joint)
    }

    // --------------- -------- -------- --------------- //
    // --------------- ELEVATOR COMMANDS --------------- //
    // --------------- -------- -------- --------------- //

    /** Sets the elevator to the preset home displacement */
    fun homeElevator(): Command {
        return elevator.setElevatorHomeDisplacement().InstantCommand(elevator)
    }

    /** Sets the elevator to the preset idle displacement */
    fun idleElevator(): Command {
        return elevator.setElevatorIdleDisplacement().InstantCommand(elevator)
    }

    /** Sets the elevator to the manual target displacement */
    fun setElevatorManualControl(): Command {
        return elevator.setElevatorManualTargetDisplacement().InstantCommand(elevator)
    }
}