package frc.tecdroid3354.systems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.tecdroid3354.commands.DriveCommands
import frc.tecdroid3354.constants.DriveMultipliers
import frc.tecdroid3354.constants.FieldConstants.FieldDimensions
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.constants.RobotTransformations
import frc.tecdroid3354.constants.SubsystemTolerances
import frc.tecdroid3354.subsystems.Flywheel.FlywheelConstants
import frc.tecdroid3354.subsystems.Hood.HoodSubsystem
import frc.tecdroid3354.subsystems.Flywheel.FlywheelSubsystem
import frc.tecdroid3354.subsystems.Hopper.HopperSubsystem
import frc.tecdroid3354.subsystems.drive.Drive
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeploySubsystem
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersSubsystem
import frc.tecdroid3354.subsystems.Tower.TowerSubsystem
import frc.tecdroid3354.subsystems.vision.Vision
import frc.tecdroid3354.utils.InstantCommand
import frc.tecdroid3354.utils.degrees
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.metersPerSecond
import frc.tecdroid3354.utils.radians
import frc.tecdroid3354.utils.radiansPerSecond
import frc.tecdroid3354.utils.simulation.FuelSim
import frc.tecdroid3354.utils.toAngle
import frc.tecdroid3354.utils.toRotation2d
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import java.math.RoundingMode
import java.text.DecimalFormat
import java.util.Optional
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.hypot

/** Inside this class, construct all control methods at robot-level (and any relevant auxiliary methods).
 * When using States, this file is responsible to control the logic flow */
class Superstructure(private val controller: CommandPS5Controller,
                     private val simDrive: SwerveDriveSimulation, private val drive: Drive,
                     private val hood: HoodSubsystem, private val flywheel: FlywheelSubsystem,
                     private val tower: TowerSubsystem, private val hopper: HopperSubsystem,
                     private val intakeDeploy: IntakeDeploySubsystem, private val intakeRollers: IntakeRollersSubsystem,
                     private val fuelSim: FuelSim, private val vision: Vision,
                     private val fieldToScoringTarget: Supplier<Translation2d>,
                     private val fieldToAssistTarget: Supplier<Translation2d>): SubsystemBase("Superstructure") {
    private val isSim: Boolean = RobotConstants.ROBOT_MODE == RobotMode.SIM // For shorter robot mode checking
    private var simFuelCount: Int = 0
    // Used to log the distance to scoring and assist targets without showing 17 or so significant figures.
    private val decimalFormaterThreePlaces: DecimalFormat = DecimalFormat("#.###") // #s after dot = number of decimals

    private val distanceToScoringTarget: Supplier<Distance> = {
        hypot(fieldToScoringTarget.get().measureX.minus(if (isSim) simDrive.simulatedDriveTrainPose.measureX else drive.pose.measureX).meters,
            fieldToScoringTarget.get().measureY.minus(if (isSim) simDrive.simulatedDriveTrainPose.measureY else drive.pose.measureY).meters)
            .meters
    }
    private val distanceToAssistTarget: Supplier<Distance> = {
        hypot(fieldToAssistTarget.get().measureX.minus(if (isSim) simDrive.simulatedDriveTrainPose.measureX else drive.pose.measureX).meters,
            fieldToAssistTarget.get().measureY.minus(if (isSim) simDrive.simulatedDriveTrainPose.measureY else drive.pose.measureY).meters)
            .meters
    }
    // To calculate radial and tangential velocities
    private val fieldRelativeSpeeds: Supplier<ChassisSpeeds> = { drive.fieldRelativeChassisSpeeds }

    // --------------- ------ - ---------- ---------- -- ------- ------ --------------- //
    // --------------- RADIAL & TANGENTIAL VELOCITIES TO SCORING TARGET --------------- //
    // --------------- ------ - ---------- ---------- -- ------- ------ --------------- //

    private val radialVelocityToScore: Supplier<LinearVelocity> = { DriveCommands.getRobotRadialVelocity(
        fieldRelativeSpeeds.get(), drive.pose,
        fieldToScoringTarget.get()
    ) }
    private val tangentialVelocityToScore: Supplier<LinearVelocity> = { DriveCommands.getRobotTangentialVelocity(
        fieldRelativeSpeeds.get(), drive.pose,
        fieldToScoringTarget.get()
    ) }

    // --------------- ------ - ---------- ---------- -- ------ ------ --------------- //
    // --------------- RADIAL & TANGENTIAL VELOCITIES TO ASSIST TARGET --------------- //
    // --------------- ------ - ---------- ---------- -- ------ ------ --------------- //

    private val radialVelocityToAssist: Supplier<LinearVelocity> = { DriveCommands.getRobotRadialVelocity(
        fieldRelativeSpeeds.get(), drive.pose,
        fieldToAssistTarget.get()
    ) }
    private val tangentialVelocityToAssist: Supplier<LinearVelocity> = { DriveCommands.getRobotTangentialVelocity(
        fieldRelativeSpeeds.get(), drive.pose,
        fieldToAssistTarget.get()
    ) }

    init {
        decimalFormaterThreePlaces.roundingMode = RoundingMode.HALF_UP
    }

    override fun periodic() {
        Logger.recordOutput("FUELS_SIM/Held_Count", simFuelCount)
        Logger.recordOutput("FUELS_SIM/Blue_HUB_Score", FuelSim.Hub.BLUE_HUB.score)
        Logger.recordOutput("FUELS_SIM/Red_HUB_Score", FuelSim.Hub.RED_HUB.score)

        Logger.recordOutput("Odometry/DistanceToHub (m)", decimalFormaterThreePlaces.format(distanceToScoringTarget.get().meters))
        Logger.recordOutput("Odometry/DistanceToAssist (m)", decimalFormaterThreePlaces.format(distanceToAssistTarget.get().meters))
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

    /** Locks the [drive] rotation to always target some point in the field. [speedLimitFactor] is between 0.0 and 1.0 */
    fun setDriveTargetingPointCommand(fieldToTarget: Supplier<Translation2d>, speedLimitFactor: Optional<Double>): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { MathUtil.applyDeadband(-controller.leftY, 0.05) * speedLimitFactor.orElse(DriveMultipliers.CONTROLLER_PRIMARY_Y_MULTIPLIER) },
            { MathUtil.applyDeadband(-controller.leftX, 0.05) * speedLimitFactor.orElse(DriveMultipliers.CONTROLLER_PRIMARY_X_MULTIPLIER) },
            { DriveCommands.getAngleFromRobotToTarget(
                drive.pose, fieldToTarget.get(),
                Optional.of(RobotTransformations.ROBOT_TO_SHOOTER.rotation.measureZ.toRotation2d()),
                true) },
        )
    }

    /** Locks the [drive] rotation to always target some heading. [speedLimitFactor] is between 0.0 and 1.0 */
    fun setDriveTargetingHeadingCommand(targetHeading: Supplier<Angle>, speedLimitFactor: Optional<Double>): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { MathUtil.applyDeadband(-controller.leftY, 0.05) * speedLimitFactor.orElse(DriveMultipliers.CONTROLLER_PRIMARY_Y_MULTIPLIER) },
            { MathUtil.applyDeadband(-controller.leftX, 0.05) * speedLimitFactor.orElse(DriveMultipliers.CONTROLLER_PRIMARY_X_MULTIPLIER) },
            { targetHeading.get().toRotation2d() },
        )
    }

    /** Calls [setDriveTargetingHeadingCommand] with the configured [fieldToScoringTarget] minus a yaw correction for SOTM.
     * Speed is limited to [DriveMultipliers.CONTROLLER_SOTM_LIMIT_MULTIPLIER] */
    fun setDriveScoreTargetingCommand(): Command {
        return setDriveTargetingHeadingCommand(
            { DriveCommands.getAngleFromRobotToTarget(
                drive.pose, fieldToScoringTarget.get(),
                Optional.of(RobotTransformations.ROBOT_TO_SHOOTER.rotation.measureZ.toRotation2d()), true)
                .minus( // Account for tangential velocity
                    getShooterYawCorrection(
                        getVirtualDistance(distanceToScoringTarget, false),
                        false
                    ).toRotation2d()
                ).toAngle() },
            Optional.of(DriveMultipliers.CONTROLLER_SOTM_LIMIT_MULTIPLIER)
        )
    }

    /** Calls [setDriveTargetingHeadingCommand] with the configured [fieldToAssistTarget] minus a yaw correction for SOTM.
     * Speed is limited to [DriveMultipliers.CONTROLLER_SOTM_LIMIT_MULTIPLIER] */
    fun setDriveAssistTargetingCommand(): Command {
        return setDriveTargetingHeadingCommand(
            { DriveCommands.getAngleFromRobotToTarget(
                drive.pose, fieldToAssistTarget.get(),
                Optional.of(RobotTransformations.ROBOT_TO_SHOOTER.rotation.measureZ.toRotation2d()), true)
                .minus( // Account for tangential velocity
                    getShooterYawCorrection(
                        getVirtualDistance(distanceToAssistTarget, true),
                        true
                    ).toRotation2d()
                ).toAngle() },
            Optional.of(DriveMultipliers.CONTROLLER_SOTM_LIMIT_MULTIPLIER)
        )
    }

    /** Checks [DriveCommands.lastDriveAngle] and compares with the current robot heading, accounting for some tolerance. */
    fun getIsDriveAtTarget(): Boolean {
        return DriveCommands.lastDriveAngle.minus(drive.rotation.toAngle()).lte(SubsystemTolerances.DRIVE_HEADING_TOLERANCE)
    }

    /** Checks if the [drive] x-coordinate is within the blue-end and red-end of the neutral zone */
    fun getIsDriveAtNeutralZone(): Boolean {
        return drive.pose.measureX.gt(FieldDimensions.NEUTRAL_ZONE_BLUE_END_X_AXIS) and
                drive.pose.measureX.lt(FieldDimensions.NEUTRAL_ZONE_RED_END_X_AXIS)
    }

    // --------------- -------- -------- --------------- //
    // --------------- SEQUENCE COMMANDS --------------- //
    // --------------- -------- -------- --------------- //

    /** A [ParallelCommandGroup] enabling shooter, waiting for [flywheel] to achieve target and then enabling
     * indexer, waiting some time and then clustering the [intakeDeploy]. Scoring polynomials are used. Allows SOTM.
     * During simulation, FUELS are also launched */
    fun setScoringSequence(): Command {
        return ParallelCommandGroup(
            setShooterScoring(),
            WaitUntilCommand({ flywheel.getIsAtTarget() })
                .andThen(setIndexerPreset())
                .andThen(WaitCommand(SubsystemTolerances.INTAKE_TIME_TOLERANCE_BEFORE_CLUSTER))
                .andThen(clusterIntakeDeploy()),
            launchSimFuel() // Has inside condition of being SIM. Won't do anything in a real robot
        )
    }

    /** A [ParallelCommandGroup] enabling shooter, waiting for [flywheel] to achieve target and then enabling
     * indexer, waiting some time and then clustering the [intakeDeploy]. Assist polynomials are used. Allows SOTM.
     * During simulation, FUELS are also launched */
    fun setAssistSequence(): Command {
        return ParallelCommandGroup(
            setShooterAssist(),
            WaitUntilCommand({ flywheel.getIsAtTarget() })
                .andThen(setIndexerPreset())
                .andThen(WaitCommand(SubsystemTolerances.INTAKE_TIME_TOLERANCE_BEFORE_CLUSTER))
                .andThen(clusterIntakeDeploy()),
            launchSimFuel() // Has inside condition of being SIM. Won't do anything in a real robot
        )
    }

    /** Stops all subsystems and extends the [intakeDeploy] */
    fun stopShootingSequenceIncludingDeploy(): Command {
        return SequentialCommandGroup(
            stopIndexer(),
            stopShooting(),
            stopIntake()
        )
    }

    /** Stops all subsystems; [intakeDeploy] is not affected */
    fun stopShootingSequenceWithoutDeploy(): Command {
        return SequentialCommandGroup(
            stopIndexer(),
            stopShooting(),
            stopIntakeRollers()
        )
    }

    // --------------- ------- -------- --------------- //
    // --------------- SHOOTER COMMANDS --------------- //
    // --------------- ------- -------- --------------- //

    /** Uses the TOF polynomials and the robot [radialVelocityToScore] or [radialVelocityToAssist],
     * all depending on whether [isAssist] */
    private fun getVirtualDistance(flywheelDistanceToTarget: Supplier<Distance>, isAssist: Boolean): Distance {
        var virtualDistance: Distance = flywheelDistanceToTarget.get()

        for (cycle in 1 .. 3) {
            virtualDistance = flywheelDistanceToTarget.get()
                .minus(
                    if (isAssist) {
                        radialVelocityToAssist.get().times(
                            flywheel.getCalculatedAssistTimeOfFlight(virtualDistance))
                    } else {
                        radialVelocityToScore.get().times(
                            flywheel.getCalculatedScoringTimeOfFlight(virtualDistance))
                    }
                )
        }

        return virtualDistance
    }

    /** Calculates the angle the FUEL will travel (delta_theta) due to tangential velocity. You can get your
     * target heading as (theta - delta_theta), where theta is the angle between robot and target when stationary.
     * @param flywheelVirtualDistanceToTarget must be passed through [getVirtualDistance]
     * @param isAssist Determines whether to use assist or scoring polynomials
     * */
    private fun getShooterYawCorrection(flywheelVirtualDistanceToTarget: Distance, isAssist: Boolean): Angle {
        val projectileVelocity: LinearVelocity = flywheelVirtualDistanceToTarget
            .div(if (isAssist) flywheel.getCalculatedAssistTimeOfFlight(flywheelVirtualDistanceToTarget)
                    else flywheel.getCalculatedScoringTimeOfFlight(flywheelVirtualDistanceToTarget))
        val tangentialVelocity: LinearVelocity = if (isAssist) tangentialVelocityToAssist.get() else tangentialVelocityToScore.get()

        return atan2(tangentialVelocity.metersPerSecond, projectileVelocity.metersPerSecond).radians
    }

    /** Enables the manual targets of [flywheel] and [hood] decided by the user in Elastic */
    fun setShooterManualControl(): Command {
        return ParallelCommandGroup(
            setFlywheelManualControl(),
            setHoodManualControl(),
        )
    }

    /** Enables [flywheel] and [hood] with their calculated scoring targets*/
    fun setShooterScoring(): Command {
        return ParallelCommandGroup(
            setFlywheelScoringTarget { getVirtualDistance(distanceToScoringTarget, false) },
            setHoodScoringTarget { getVirtualDistance(distanceToScoringTarget, false) },
        )
    }

    /** Enables [flywheel] and [hood] with their calculated assist targets*/
    fun setShooterAssist(): Command {
        return ParallelCommandGroup(
            setFlywheelAssistTarget { getVirtualDistance(distanceToAssistTarget, true) },
            setHoodAssistTarget { getVirtualDistance(distanceToAssistTarget, true) },
        )
    }

    /** [SequentialCommandGroup] stoping the [flywheel] and homing the [hood] */
    fun stopShooting(): Command {
        return ParallelCommandGroup(
            stopFlywheel(),
            homeHood(),
        )
    }

    // --------------- ------- -------- --------------- //
    // --------------- INDEXER COMMANDS --------------- //
    // --------------- ------- -------- --------------- //

    /** Enables the manual targets of [tower] and [hopper] decided by the user in Elastic */
    fun setIndexerManualControl(): Command {
        return ParallelCommandGroup(
            enableTowerManualControl(),
            enableHopperManualControl()
        )
    }

    /** Enables the preset targets for both [tower] and [hopper] */
    fun setIndexerPreset(): Command {
        return ParallelCommandGroup(
            enableTowerPreset(),
            enableHopperPreset()
        )
    }

    /** Stops the [tower] and [hopper] */
    fun stopIndexer(): Command {
        return ParallelCommandGroup(
            stopTower(),
            stopHopper()
        )
    }

    // --------------- ------ -------- --------------- //
    // --------------- INTAKE COMMANDS --------------- //
    // --------------- ------ -------- --------------- //

    /** Enables the manual targets of [intakeDeploy] and [intakeRollers] decided by the user in Elastic */
    fun setIntakeManualControl(): Command {
        return ParallelCommandGroup(
            setIntakeDeployManualControl(),
            enableIntakeRollersManualControl()
        )
    }

    /** Normal teleop intake control. Deploys the [intakeDeploy] and enables the [intakeRollers] */
    fun setIntakeTeleopControl(): Command {
        return SequentialCommandGroup(
            extendIntakeDeploy(),
            enableIntakeRollersPreset()
        )
    }

    /** Stops the [intakeRollers]. [intakeDeploy] is called to fully extend, in case it was clustered. */
    fun stopIntake(): Command {
        return SequentialCommandGroup(
            extendIntakeDeploy(),
            stopIntakeRollers()
        )
    }

    /** Stops the [intakeRollers] and clusters the [intakeDeploy]. Motion is NOT repeated. */
    fun idleIntake(): Command {
        return ParallelCommandGroup(
            stopIntakeRollers(),
            clusterIntakeDeploy()
        )
    }

    /** Stops the [intakeRollers] and homes the [intakeDeploy]. */
    fun homeIntake(): Command {
        return ParallelCommandGroup(
            stopIntakeRollers(),
            homeIntakeDeploy()
        )
    }

    /** For simulation purposes. It returns true if the intake is deployed, its rollers are active and hopper is not at full capacity */
    fun getIsAbleToIntake(): Boolean {
        return (intakeDeploy.getIsDeployed() and intakeRollers.getIsActive()) and
                (simFuelCount < SubsystemTolerances.SIMULATION_HOPPER_FUEL_CAPACITY_TOLERANCE)
    }

    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- SIMULATION SUBSYSTEM COMMANDS ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/

    /** Made to match the fuel velocity calculation of the Trajectory Simulation App */
    private fun getFuelSimLaunchVelocity(): LinearVelocity {
        return flywheel.getFlywheelVelocity().radiansPerSecond
            .div(FlywheelConstants.Mechanical.REDUCTION.getRatio())
            .times(FlywheelConstants.Mechanical.ROLLER_DIAMETER.div(2.0).meters)
            .times(FlywheelConstants.Mechanical.ESTIMATED_ROLLER_EFFICIENCY)
            .metersPerSecond
    }

    /** Launches a FUEL in simulation. Some delay between launches is configured in [SubsystemTolerances].
     * Verification checks for having available FUEL in hopper and flywheel being at target are included.
     * This will do nothing in a real robot. */
    fun launchSimFuel(): Command {
        if (isSim.not()) return Commands.none()

        return SequentialCommandGroup(
            WaitUntilCommand({ (simFuelCount > 0) and flywheel.getIsAtTarget() }), // Continuously checks if it can shoot
            InstantCommand({ // Decreases fuel count and launches one fuel
                decreaseHopperFuelCount(1)
                fuelSim.launchFuel(
                    getFuelSimLaunchVelocity(),
                    90.0.degrees.minus(hood.getHoodPosition()),
                    0.0.degrees, // This is for turrets. Robot rotation and shooter transformations are inside FuelSim.java
                    RobotTransformations.ROBOT_TO_SHOOTER.measureZ // Starting height
                )
            }),
            WaitCommand(SubsystemTolerances.SIMULATION_FUEL_LAUNCH_TIMEOUT_TOLERANCE)
        ).repeatedly()
    }

    /** **SIMULATION ONLY**. It increases the virtually held FUELS by one */
    fun increaseHopperFuelCount() {
        simFuelCount += 1
    }

    /** **SIMULATION ONLY**. It decreases the virtually held FUELS by [decreaseCount], one by default.
     *
     * If [decreaseCount] is less than 0, no action will be performed.
     *
     * If [simFuelCount] is less than 0 after applying [decreaseCount], it will be reassigned to zero. */
    fun decreaseHopperFuelCount(decreaseCount: Int = 1) {
        if (decreaseCount < 0) return // Prevent adding fuels here
        simFuelCount -= decreaseCount

        if (simFuelCount < 0) simFuelCount = 0 // Can't contain less than 0 fuel
    }

    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- INDIVIDUAL SUBSYSTEM COMMANDS ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/
    /** <--------------- ---------- --------- -------- ---------------> **/

    // --------------- ---- -------- --------------- //
    // --------------- HOOD COMMANDS --------------- //
    // --------------- ---- -------- --------------- //

    /** Sets the hood to the preset home displacement */
    fun homeHood(): Command {
        return hood.setHoodHomePosition().InstantCommand(hood)
    }

    /** Calculates target Angle based on [flywheelDistanceToTarget] and scoring polynomial coefficients */
    fun setHoodScoringTarget(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return hood.setHoodScoringPosition(flywheelDistanceToTarget)
    }

    /** Calculates target Angle based on [flywheelDistanceToTarget] and assist polynomial coefficients */
    fun setHoodAssistTarget(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return hood.setHoodAssistPosition(flywheelDistanceToTarget)
    }

    /** Sets the hood to the manual target displacement */
    fun setHoodManualControl(): Command {
        return hood.setHoodManualPosition().InstantCommand(hood)
    }

    // --------------- -------- -------- --------------- //
    // --------------- FLYWHEEL COMMANDS --------------- //
    // --------------- -------- -------- --------------- //

    /** Calculates target RPMs based on [flywheelDistanceToTarget] and scoring polynomial coefficients */
    fun setFlywheelScoringTarget(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return flywheel.enableFlywheelCalculatedScoringVelocity(flywheelDistanceToTarget)
    }

    /** Calculates target RPMs based on [flywheelDistanceToTarget] and assist polynomial coefficients */
    fun setFlywheelAssistTarget(flywheelDistanceToTarget: Supplier<Distance>): Command {
        return flywheel.enableFlywheelCalculatedAssistVelocity(flywheelDistanceToTarget)
    }

    /** Takes the manual velocity set by the user in Elastic */
    fun setFlywheelManualControl(): Command {
        return flywheel.enableFlywheelManualVelocity().InstantCommand(flywheel)
    }

    /** Stops the flywheel */
    fun stopFlywheel(): Command {
        return flywheel.stopFlywheel().InstantCommand(flywheel)
    }

    // --------------- ------- -------- --------------- //
    // --------------- INDEXER COMMANDS --------------- //
    // --------------- ------- -------- --------------- //

    /** Enables the preset target of the [tower] */
    fun enableTowerPreset(): Command {
        return tower.enableTowerPresetVelocity().InstantCommand(tower)
    }

    /** Enables the manual target of the [tower] */
    fun enableTowerManualControl(): Command {
        return tower.enableTowerManualVelocity().InstantCommand(tower)
    }

    /** Stops the tower */
    fun stopTower(): Command {
        return tower.stopTower().InstantCommand(tower)
    }

    /** Enables the preset target of the [hopper] */
    fun enableHopperPreset(): Command {
        return hopper.enableHopperPresetVelocity().InstantCommand(hopper)
    }

    /** Enables the manual target of the [tower] */
    fun enableHopperManualControl(): Command {
        return hopper.enableHopperManualVelocity().InstantCommand(hopper)
    }

    /** Stops the tower */
    fun stopHopper(): Command {
        return hopper.stopHopper().InstantCommand(hopper)
    }

    // --------------- ------ ------ -------- --------------- //
    // --------------- INTAKE DEPLOY COMMANDS --------------- //
    // --------------- ------ ------ -------- --------------- //

    /** Sets the deploy to the preset extended displacement */
    fun extendIntakeDeploy(): Command {
        return intakeDeploy.setIntakeDeployExtendedDisplacement().InstantCommand(intakeDeploy)
    }
    /** Sets the deploy to the preset clustering displacement */
    fun clusterIntakeDeploy(): Command {
        return intakeDeploy.setIntakeDeployClusteringDisplacement().InstantCommand(intakeDeploy)
    }

    /** Sets the elevator to the preset home displacement */
    fun homeIntakeDeploy(): Command {
        return intakeDeploy.setIntakeDeployHomeDisplacement().InstantCommand(intakeDeploy)
    }

    /** Sets the deploy to the manual target displacement */
    fun setIntakeDeployManualControl(): Command {
        return intakeDeploy.setIntakeDeployManualTargetDisplacement().InstantCommand(intakeDeploy)
    }

    // --------------- ------ ------- -------- --------------- //
    // --------------- INTAKE ROLLERS COMMANDS --------------- //
    // --------------- ------ ------- -------- --------------- //

    /** Enables the preset target of the [intakeRollers] */
    fun enableIntakeRollersPreset(): Command {
        return intakeRollers.enableIntakeRollersPresetVelocity().InstantCommand(intakeRollers)
    }

    /** Enables the manual target of the [intakeRollers] */
    fun enableIntakeRollersManualControl(): Command {
        return intakeRollers.enableIntakeRollersManualVelocity().InstantCommand(intakeRollers)
    }

    /** Stops the intake rollers */
    fun stopIntakeRollers(): Command {
        return intakeRollers.stopIntakeRollers().InstantCommand(intakeRollers)
    }

    // --------------- ----- - ----- -------- --------------- //
    // --------------- COAST & BRAKE COMMANDS --------------- //
    // --------------- ----- - ----- -------- --------------- //

    /** Coasts [hood] and [intakeDeploy] */
    fun coastSubsystems(): Command {
        return ParallelCommandGroup(
                hood.coastHood(),
                intakeDeploy.coastIntakeDeployMotors(),
            )
    }

    /** Brakes [hood] and [intakeDeploy] */
    fun brakeSubsystems(): Command {
        return ParallelCommandGroup(
            hood.brakeHood(),
            intakeDeploy.brakeIntakeDeployMotors(),
        )
    }
}