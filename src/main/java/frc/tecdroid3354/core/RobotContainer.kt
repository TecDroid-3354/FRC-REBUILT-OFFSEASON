package frc.tecdroid3354.core

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.tecdroid3354.RobotVisualizer
import frc.tecdroid3354.commands.DriveCommands
import frc.tecdroid3354.constants.FieldConstants.TargetTranslations
import frc.tecdroid3354.constants.FieldConstants.FieldDimensions
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotConstants.IS_RED_ALLIANCE
import frc.tecdroid3354.constants.RobotDimensions
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.constants.SubsystemTolerances
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.generated.SwerveTunerConstants
import frc.tecdroid3354.subsystems.Hood.HoodIO
import frc.tecdroid3354.subsystems.Hood.HoodIOSim
import frc.tecdroid3354.subsystems.Hood.HoodIOTalonFX
import frc.tecdroid3354.subsystems.Hood.HoodSubsystem
import frc.tecdroid3354.subsystems.Flywheel.FlywheelIO
import frc.tecdroid3354.subsystems.Flywheel.FlywheelIOSim
import frc.tecdroid3354.subsystems.Flywheel.FlywheelIOTalonFX
import frc.tecdroid3354.subsystems.Flywheel.FlywheelSubsystem
import frc.tecdroid3354.subsystems.Hopper.HopperIO
import frc.tecdroid3354.subsystems.Hopper.HopperIOSim
import frc.tecdroid3354.subsystems.Hopper.HopperIOTalonFX
import frc.tecdroid3354.subsystems.Hopper.HopperSubsystem
import frc.tecdroid3354.subsystems.drive.*
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployIO
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployIOSim
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeployIOTalonFX
import frc.tecdroid3354.subsystems.IntakeDeploy.IntakeDeploySubsystem
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersIO
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersIOSim
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersIOTalonFX
import frc.tecdroid3354.subsystems.IntakeRollers.IntakeRollersSubsystem
import frc.tecdroid3354.subsystems.Tower.TowerIO
import frc.tecdroid3354.subsystems.Tower.TowerIOSim
import frc.tecdroid3354.subsystems.Tower.TowerIOTalonFX
import frc.tecdroid3354.subsystems.Tower.TowerSubsystem
import frc.tecdroid3354.subsystems.vision.*
import frc.tecdroid3354.systems.Superstructure
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.simulation.FuelSim
import frc.tecdroid3354.utils.simulation.RobotBumpSim
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.Optional

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    private val driverController = CommandPS5Controller(RobotConstants.MAIN_CONTROLLER_PORT)
    // ALL interactions (with maple sim drive exception) with subsystems will be done through this object.
    private val superstructure: Superstructure

    private val autoChooser: LoggedDashboardChooser<Command>

    private lateinit var drive                  : Drive
    private lateinit var mapleSimDrive          : SwerveDriveSimulation
    private lateinit var bumpSim                : RobotBumpSim
    private lateinit var fuelSim                : FuelSim
    // The 'false' parameter prevents field bumps from being treated as obstacles (for RobotBumpSim to work)
    val simField                                : SimulatedArena = Arena2026Rebuilt(false)

    private lateinit var vision                 : Vision

    private lateinit var hoodSubsystem          : HoodSubsystem
    private lateinit var flywheelSubsystem      : FlywheelSubsystem
    private lateinit var towerSubsystem         : TowerSubsystem
    private lateinit var hopperSubsystem        : HopperSubsystem
    private lateinit var intakeDeploySubsystem  : IntakeDeploySubsystem
    private lateinit var intakeRollersSubsystem : IntakeRollersSubsystem

    lateinit var robotVisualizer                : RobotVisualizer

    /** Makes sure everything is initialized and configured */
    init
    {
        initializeSubsystems() // This method MUST be the first one called, otherwise you'll be accessing null objects

        superstructure = Superstructure( // Constructs the superstructure with initialized subsystems
            driverController,
            mapleSimDrive, drive,
            hoodSubsystem, flywheelSubsystem,
            towerSubsystem, hopperSubsystem,
            intakeDeploySubsystem, intakeRollersSubsystem,
            fuelSim, vision,
            { if (IS_RED_ALLIANCE.asBoolean) TargetTranslations.RED_HUB else TargetTranslations.BLUE_HUB }, // Score Target
            { if (IS_RED_ALLIANCE.asBoolean) { // Decides Red Upper / Bottom Assist Target
                if (drive.pose.measureY.gte(FieldDimensions.Y_AXIS_HEIGHT.div(2.0)))
                    TargetTranslations.UPPER_RED_ASSIST else TargetTranslations.BOTTOM_RED_ASSIST
            } else { // Decides Blue Upper / Bottom Assist Target
                if (drive.pose.measureY.gte(FieldDimensions.Y_AXIS_HEIGHT.div(2.0)))
                    TargetTranslations.UPPER_BLUE_ASSIST else TargetTranslations.BOTTOM_BLUE_ASSIST
            } }
        )

        configureFuelSim()

        registerNamedCommandsInit() // Must be called before autoChooser binds the autonomous routines

        // Initializes auto chooser after AutoBuilder was configured inside Drive.
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        configureAutonomousCommands()   // Must be called after autoChooser was initialized
        configureBindings()             // Must be called after superstructure was initialized

    }

    /** For PathPlanner Named Commands. Called during init, **BEFORE** [autoChooser] is initialized */
    private fun registerNamedCommandsInit() {
        NamedCommands.registerCommand("Score", superstructure.setDriveScoreTargetingCommand()
            .alongWith(
                WaitUntilCommand({ superstructure.getIsDriveAtTarget() })
                    .andThen(superstructure.setScoringSequence()))
            .withTimeout(SubsystemTolerances.AUTONOMOUS_SCORING_SEQUENCE_TIME_TOLERANCE))
        NamedCommands.registerCommand("Enable_Intake", superstructure.setIntakeTeleopControl())
        NamedCommands.registerCommand("Disable_Intake", superstructure.stopIntake())
    }

    /** Configurations that must be applied every time the robot is enabled, regardless of mode / phase */
    fun robotEnabledConfig() {
        superstructure.stopShootingSequenceWithoutDeploy()
    }

    /** Configurations that must be applied every time the robot is disabled */
    fun robotDisabledConfig() {
        superstructure.stopShootingSequenceWithoutDeploy()
    }

    /** Configurations that must be applied at the start of teleop */
    fun robotTeleopInitConfig() {
        // Prevents the chassis from rotating to 0deg after the autonomous period
        DriveCommands.overrideLastJoystickAngle(drive.rotation)

        superstructure.setDriveDefaultCommand()
    }

    /** Configurations that must be applied at the start of autonomous */
    fun robotAutonomousInitConfig() {
        superstructure.removeDriveDefaultCommand()
    }

    /**
     * Adds every PathPlanner / Choreo auto to the auto chooser
     */
    private fun configureAutonomousCommands() {
        // If not configured, the autoChooser may return Null inside getAutonomousCommand()
        autoChooser.addDefaultOption("None", Commands.none())

        autoChooser.addOption("One_Sweep_Test", AutoBuilder.buildAuto("Left_One_Sweep"))
        autoChooser.addOption("Two_Sweep_Test", AutoBuilder.buildAuto("Left_Two_Sweep"))
    }

    /**
     * Configures the commands for [driverController]. Every subsystem interaction is done through [superstructure].
     */
    private fun configureBindings() {
        driverController.options() // Reset odometry heading
            .onTrue(superstructure.resetOdometryHeading(Optional.empty()))

        driverController.L2() // Drive heading locked to assist / scoring target. Allows SOTM.
            .whileTrue(Commands.select( // Takes a map of boolean -> Command, where true is assist and false is score.
                mapOf<Boolean, Command>(
                    true to superstructure.setDriveAssistTargetingCommand(),
                    false to superstructure.setDriveScoreTargetingCommand()
                )
            ) { superstructure.getIsDriveAtNeutralZone() }) // If this is true, locks to assist, if not locks to score.
            .onFalse(superstructure.setDriveTeleopCommand())

        driverController.R2() // Shooting sequences with assist / scoring polynomials. Allows SOTM.
            .whileTrue(Commands.select( // Takes a map of boolean -> Command, where true is assist and false is score.
                mapOf<Boolean, Command>(
                    true to superstructure.setAssistSequence(),
                    false to superstructure.setScoringSequence()
                )
            ) { superstructure.getIsDriveAtNeutralZone() }) // If this is true, launches to assist, if not launches to score.
            .onFalse(superstructure.stopShootingSequenceIncludingDeploy())

        driverController.R1()
            .whileTrue(superstructure.setIntakeTeleopControl())
            .onFalse(superstructure.stopIntake())

        driverController.L1()
            .whileTrue(superstructure.clusterIntakeDeploy())
            .onFalse(superstructure.stopIntake())

        driverController.square() // Shooter (Hood + Flywheel) manual control
            .whileTrue(superstructure.setShooterManualControl())
            .onFalse(superstructure.stopShooting())

        driverController.triangle() // Indexer (Tower + Hopper) manual control
            .whileTrue(superstructure.setIndexerManualControl())
            .onFalse(superstructure.stopIndexer())

        driverController.circle() // Intake (Deploy + Rollers) manual control
            .whileTrue(superstructure.setIntakeManualControl())
            .onFalse(superstructure.stopIntake())

//        driverController.L1() // Shooter (Hood + Flywheel) calculated scoring targets
//            .whileTrue(superstructure.setShooterScoring())
//            .onFalse(superstructure.stopShooting())

        driverController.povLeft() // Launches FUEL. SIMULATION ONLY.
            .onTrue(superstructure.launchSimFuel())

        driverController.povUp() // Coasts hood and deploy regardless of being disabled
            .onTrue(superstructure.coastSubsystems().ignoringDisable(true))

        driverController.povDown() // Brakes hood and deploy regardless of being disabled
            .onTrue(superstructure.brakeSubsystems().ignoringDisable(true))



    }

    /** Returns the selected command in [autoChooser] */
    fun getAutonomousCommand(): Command {
        return autoChooser.get()
    }

    // --------------- ---------- -------- --------------- //
    // --------------- SIMULATION SPECIFIC --------------- //
    // --------------- ---------- -------- --------------- //

    /** Additional configuration (robot and intake) of [fuelSim]. It also starts the simulation. */
    private fun configureFuelSim() {
        fuelSim.registerRobot( // Registers the dimensions and speeds of the robot for accurate simulation
            RobotDimensions.ROBOT_WIDTH,
            RobotDimensions.ROBOT_LENGTH,
            RobotDimensions.BUMPERS_HEIGHT,
            { mapleSimDrive.simulatedDriveTrainPose },
            { drive.fieldRelativeChassisSpeeds }
        )
        fuelSim.registerIntake( // Registers the robot intake for accurate interaction
            (RobotDimensions.ROBOT_WIDTH.minus(RobotDimensions.BUMPERS_DEPTH)).div(2.0),
            (RobotDimensions.ROBOT_WIDTH.minus(RobotDimensions.BUMPERS_DEPTH)).div(2.0)
                .plus(SubsystemsMovementLimits.INTAKE_DEPLOY_DISPLACEMENT_LIMITS.maximum),
            RobotDimensions.ROBOT_LENGTH.div(2.0).unaryMinus(),
            RobotDimensions.ROBOT_LENGTH.div(2.0),
            { superstructure.getIsAbleToIntake() }, // Will only intake when this returns true
            { superstructure.increaseHopperFuelCount() } // Will call this function when a fuel is intaked
        )
        fuelSim.start()
        fuelSim.enableAirResistance()
    }

    /** Resets simulation odometry and field */
    fun resetSimulation() {
        if (RobotConstants.ROBOT_MODE != RobotMode.SIM) return

        superstructure.resetOdometryPose(Pose2d(3.0, 3.0, Rotation2d()))
        fuelSim.clearFuel()
        fuelSim.spawnStartingFuel()

        FuelSim.Hub.BLUE_HUB.resetScore()
        FuelSim.Hub.RED_HUB.resetScore()
    }

    /** Updates [simField] and [mapleSimDrive], taking [bumpSim] into account. Poses logged with [Logger] */
    fun updateSimulation() {
        if (RobotConstants.ROBOT_MODE != RobotMode.SIM) return
        simField.simulationPeriodic() // Must be called first
        fuelSim.updateSim() // Updates FUEL simulation

        // Physics update accounting for bump (must call after maple sim updates)
        val robotPose2d = mapleSimDrive.simulatedDriveTrainPose
        val fieldRelativeSpeeds = mapleSimDrive.driveTrainSimulatedChassisSpeedsFieldRelative

        val robotPose3d = bumpSim.update(robotPose2d, fieldRelativeSpeeds, 5)

        // Only override maple sim pose if over bump
        if (bumpSim.isOnRamp) {
            mapleSimDrive.setSimulationWorldPose(bumpSim.getSimWorldPose(robotPose2d))
        }

        // Log the robot and game pieces
        Logger.recordOutput("FieldSimulation/RobotPosition2d", mapleSimDrive.simulatedDriveTrainPose)
        Logger.recordOutput("FieldSimulation/RobotPosition3d", robotPose3d) // View this one in AdvantageScope
    }

    // --------------- --------- -------------- --------------- //
    // --------------- SUBSYSTEM INITIALIZATION --------------- //
    // --------------- --------- -------------- --------------- //

    /**
     * Initializes all subsystems with the corresponding IOLayer, depending on [RobotConstants.ROBOT_MODE].
     *
     * [robotVisualizer] is also initialized here, unlike [superstructure] and [autoChooser] that are initialized in init.
     */
    private fun initializeSubsystems() {
        when(RobotConstants.ROBOT_MODE) {

            RobotMode.REAL -> { // Sim objects won't do anything
                mapleSimDrive = SwerveDriveSimulation(
                    Drive.getMapleSimConfig(),
                    Pose2d(3.0.meters, 3.0.meters, Rotation2d())
                )
                simField.addDriveTrainSimulation(mapleSimDrive)

                fuelSim = FuelSim("FUELS_SIM")

                bumpSim = RobotBumpSim(Drive.getModuleTranslations())

                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFX(SwerveTunerConstants.FrontLeft), ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
                    ModuleIOTalonFX(SwerveTunerConstants.BackLeft), ModuleIOTalonFX(SwerveTunerConstants.BackRight),
                    {}
                )
                vision = Vision(
                    drive,
                    VisionIOLimelight(VisionConstants.leftCameraName, drive::getRotation),
                    VisionIOLimelight(VisionConstants.rightCameraName, drive::getRotation),
                    VisionIOLimelight(VisionConstants.backCameraName, drive::getRotation),
                )

                hoodSubsystem = HoodSubsystem(HoodIOTalonFX())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIOTalonFX())
                towerSubsystem = TowerSubsystem(TowerIOTalonFX())
                hopperSubsystem = HopperSubsystem(HopperIOTalonFX())
                intakeDeploySubsystem = IntakeDeploySubsystem(IntakeDeployIOTalonFX())
                intakeRollersSubsystem = IntakeRollersSubsystem(IntakeRollersIOTalonFX())
            }

            RobotMode.SIM -> {
                // Drive-specific and extra simulation objects (maple-sim, simField, fuelSim and bumpSim)
                mapleSimDrive = SwerveDriveSimulation(
                    Drive.getMapleSimConfig(),
                    Pose2d(3.0.meters, 3.0.meters, Rotation2d())
                )
                simField.addDriveTrainSimulation(mapleSimDrive)

                fuelSim = FuelSim("FUELS_SIM")

                bumpSim = RobotBumpSim(Drive.getModuleTranslations())

                // Normal Simulation Layer of all subsystems.
                drive = Drive(
                    GyroIOSim(mapleSimDrive.gyroSimulation),
                    ModuleIOSim(mapleSimDrive.modules[0]), ModuleIOSim(mapleSimDrive.modules[1]),
                    ModuleIOSim(mapleSimDrive.modules[2]), ModuleIOSim(mapleSimDrive.modules[3]),
                    mapleSimDrive::setSimulationWorldPose
                )

                // Normal subsystems simulation
                vision = Vision(
                    drive,
                    VisionIOPhotonVisionSim(VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera, mapleSimDrive::getSimulatedDriveTrainPose),
                    VisionIOPhotonVisionSim(VisionConstants.rightCameraName, VisionConstants.robotToRightCamera, mapleSimDrive::getSimulatedDriveTrainPose),
                    VisionIOPhotonVisionSim(VisionConstants.backCameraName, VisionConstants.robotToBackCamera, mapleSimDrive::getSimulatedDriveTrainPose),
                )

                hoodSubsystem = HoodSubsystem(HoodIOSim())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIOSim())
                towerSubsystem = TowerSubsystem(TowerIOSim())
                hopperSubsystem = HopperSubsystem(HopperIOSim())
                intakeDeploySubsystem = IntakeDeploySubsystem(IntakeDeployIOSim())
                intakeRollersSubsystem = IntakeRollersSubsystem(IntakeRollersIOSim())
            }

            RobotMode.REPLAY -> { // Sim objects won't do anything
                mapleSimDrive = SwerveDriveSimulation(
                    Drive.getMapleSimConfig(),
                    Pose2d(3.0.meters, 3.0.meters, Rotation2d())
                )
                simField.addDriveTrainSimulation(mapleSimDrive)

                fuelSim = FuelSim("FUELS_SIM")

                bumpSim = RobotBumpSim(Drive.getModuleTranslations())

                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {}, object : ModuleIO {}, object : ModuleIO {}, object : ModuleIO {},
                    {}
                )
                // One dummy IO for each camera on the robot
                vision = Vision(drive, object : VisionIO {}, object : VisionIO {}, object : VisionIO {})

                hoodSubsystem = HoodSubsystem(HoodIO.DummyHoodIO())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIO.DummyFlywheelIO())
                towerSubsystem = TowerSubsystem(TowerIO.DummyTowerIO())
                hopperSubsystem = HopperSubsystem(HopperIO.DummyHopperIO())
                intakeDeploySubsystem = IntakeDeploySubsystem(IntakeDeployIO.DummyIntakeDeployIO())
                intakeRollersSubsystem = IntakeRollersSubsystem(IntakeRollersIO.DummyIntakeRollersIO())
            }
        }

        robotVisualizer = RobotVisualizer( // Independent of robot mode. Initialized at last to give it the parameters.
            { hoodSubsystem.getHoodPosition() },
            { intakeDeploySubsystem.getIntakeDeployDisplacement() }
        )
    }
}