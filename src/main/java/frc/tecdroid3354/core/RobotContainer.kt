package frc.tecdroid3354.core

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.tecdroid3354.RobotVisualizer
import frc.tecdroid3354.commands.DriveCommands
import frc.tecdroid3354.constants.FieldConstants.BoundaryLimits
import frc.tecdroid3354.constants.FieldConstants.TargetTranslations
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotConstants.IS_RED_ALLIANCE
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.generated.SwerveTunerConstants
import frc.tecdroid3354.subsystems.angularPosition.JointIO
import frc.tecdroid3354.subsystems.angularPosition.JointIOSim
import frc.tecdroid3354.subsystems.angularPosition.JointIOTalonFX
import frc.tecdroid3354.subsystems.angularPosition.JointSubsystem
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelIO
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelIOSim
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelIOTalonFX
import frc.tecdroid3354.subsystems.angularVelocity.FlywheelSubsystem
import frc.tecdroid3354.subsystems.drive.*
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorIO
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorIOSim
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorIOTalonFX
import frc.tecdroid3354.subsystems.linearDisplacement.ElevatorSubsystem
import frc.tecdroid3354.subsystems.vision.*
import frc.tecdroid3354.systems.Superstructure
import frc.tecdroid3354.utils.meters
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
    private lateinit var physicsBumpSim         : RobotBumpSim
    // The 'false' parameter prevents field bumps from being treated as obstacles (for RobotBumpSim to work)
    val simField                                : SimulatedArena = Arena2026Rebuilt(false)

    private lateinit var vision                 : Vision

    private lateinit var jointSubsystem         : JointSubsystem
    private lateinit var elevatorSubsystem      : ElevatorSubsystem
    private lateinit var flywheelSubsystem      : FlywheelSubsystem

    lateinit var robotVisualizer                : RobotVisualizer

    /** Makes sure everything is initialized and configured */
    init
    {
        initializeSubsystems() // This method MUST be the first one called, otherwise you'll be accessing null objects

        // Initializes auto chooser after AutoBuilder was configured inside Drive.
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        superstructure = Superstructure( // Constructs the superstructure with initialized subsystems
            driverController,
            mapleSimDrive, drive,
            jointSubsystem, elevatorSubsystem, flywheelSubsystem,
            vision
        )

        configureAutonomousCommands()   // Must be called after autoChooser was initialized
        configureBindings()             // Must be called after superstructure was initialized

    }

    /** For PathPlanner Named Commands. Called during [Robot] init */
    fun registerNamedCommandsInit() {

    }

    /** Configurations that must be applied every time the robot is enabled, regardless of mode / phase */
    fun robotEnabledConfig() {

    }

    /** Configurations that must be applied every time the robot is disabled */
    fun robotDisabledConfig() {

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
    private fun configureBindings() { // Would leave better commands in template, but I'm using a keyboard.
        if (RobotConstants.ROBOT_MODE == RobotMode.SIM) {
            Trigger{ (BoundaryLimits.BLUE_TOWER_BOUNDARY.contains(mapleSimDrive.simulatedDriveTrainPose.translation)
                    || BoundaryLimits.RED_TOWER_BOUNDARY.contains(mapleSimDrive.simulatedDriveTrainPose.translation))
                    && DriverStation.isTeleop() }
                .whileTrue(superstructure.idleArm())
                .onFalse(superstructure.homeArm())
        }

        driverController.options() // Equivalent to Xbox .start()
            .onTrue(superstructure.resetOdometryHeading(Optional.empty()))

        driverController.square() // Equivalent to Xbox .a()
            .whileTrue(superstructure.setArmManualControl())
            .onFalse(superstructure.homeArm())

        driverController.L2() // Equivalent to .leftTrigger()
            .whileTrue(superstructure.setDriveTargetingCommand(
                { if (IS_RED_ALLIANCE.asBoolean) TargetTranslations.RED_HUB else TargetTranslations.BLUE_HUB }
            ))
            .onFalse(superstructure.setDriveTeleopCommand())
    }

    /** Returns the selected command in [autoChooser] */
    fun getAutonomousCommand(): Command {
        return autoChooser.get()
    }

    // --------------- ---------- -------- --------------- //
    // --------------- SIMULATION SPECIFIC --------------- //
    // --------------- ---------- -------- --------------- //

    /** Resets simulation odometry and field */
    fun resetSimulation() {
        if (RobotConstants.ROBOT_MODE != RobotMode.SIM) return

        superstructure.resetOdometryPose(Pose2d(3.0, 3.0, Rotation2d()))
        simField.resetFieldForAuto()
    }

    /** Updates [simField] and [mapleSimDrive], taking [physicsBumpSim] into account. Poses logged with [Logger] */
    fun updateSimulation() {
        if (RobotConstants.ROBOT_MODE != RobotMode.SIM) return
        simField.simulationPeriodic() // Must be called first

        // Physics update accounting for bump (must call after maple sim updates)
        val robotPose2d = mapleSimDrive.simulatedDriveTrainPose
        val fieldRelativeSpeeds = mapleSimDrive.driveTrainSimulatedChassisSpeedsFieldRelative

        val robotPose3d = physicsBumpSim.update(robotPose2d, fieldRelativeSpeeds, 5)

        // Only override maple sim pose if over ramp
        if (physicsBumpSim.isOnRamp) {
            mapleSimDrive.setSimulationWorldPose(physicsBumpSim.getSimWorldPose(robotPose2d))
        }

        // Log the robot and game pieces
        Logger.recordOutput("FieldSimulation/RobotPosition2d", mapleSimDrive.simulatedDriveTrainPose)
        Logger.recordOutput("FieldSimulation/RobotPosition3d", robotPose3d) // View this one in AdvantageScope
        Logger.recordOutput(
            "FieldSimulation/Fuel",
            *simField.getGamePiecesArrayByType("Fuel")
        )
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

            RobotMode.REAL -> {
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

                jointSubsystem = JointSubsystem(JointIOTalonFX())
                elevatorSubsystem = ElevatorSubsystem(ElevatorIOTalonFX())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIOTalonFX())
            }

            RobotMode.SIM -> {
                // Drive-specific simulation (maple-sim and RobotBumpSim)
                mapleSimDrive = SwerveDriveSimulation(
                    Drive.getMapleSimConfig(),
                    Pose2d(3.0.meters, 3.0.meters, Rotation2d())
                )
                simField.addDriveTrainSimulation(mapleSimDrive)

                physicsBumpSim = RobotBumpSim(Drive.getModuleTranslations())

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

                jointSubsystem = JointSubsystem(JointIOSim())
                elevatorSubsystem = ElevatorSubsystem(ElevatorIOSim())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIOSim())
            }

            RobotMode.REPLAY -> {
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {}, object : ModuleIO {}, object : ModuleIO {}, object : ModuleIO {},
                    {}
                )
                // One dummy IO for each camera on the robot
                vision = Vision(drive, object : VisionIO {}, object : VisionIO {}, object : VisionIO {})

                jointSubsystem = JointSubsystem(JointIO.DummyJointIO())
                elevatorSubsystem = ElevatorSubsystem(ElevatorIO.DummyElevatorIO())
                flywheelSubsystem = FlywheelSubsystem(FlywheelIO.DummyFlywheelIO())
            }
        }

        robotVisualizer = RobotVisualizer( // Independent of robot mode. Initialized at last to give it the parameters.
            { jointSubsystem.getJointPosition() },
            { elevatorSubsystem.getElevatorDisplacement() }
        )
    }
}