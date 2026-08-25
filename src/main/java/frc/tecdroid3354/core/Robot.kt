package frc.tecdroid3354.core


import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.tecdroid3354.constants.FieldConstants
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotMode.REAL
import frc.tecdroid3354.constants.RobotMode.SIM
import frc.tecdroid3354.constants.RobotMode.REPLAY
import frc.tecdroid3354.constants.RobotTelemetry
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePiece
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null

    init {
        initLogging()
    }

    override fun robotInit() {
        // For PathPlanner Auto Building to work with Named Commands. Must be called during robot init.
        RobotContainer.registerNamedCommandsInit()

        // Just silences the annoying joystick unplugged warning
        DriverStation.silenceJoystickConnectionWarning(true)

        // Warms up PathPlanner, as the first run a path might have significantly more delay than subsequent runs.
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand())
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand())

        // Start Elastic
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath())

        // Just to check if the robot mode is correctly set
        Logger.recordOutput(RobotTelemetry.ROBOT_MODE_TAB, RobotConstants.ROBOT_MODE.toString())
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // TODO(1): Comment this line if loop cycle is NOT significantly less than 20ms
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.

        CommandScheduler.getInstance().run()

        // Updates the 2D and 3D (if applicable) visualization of all subsystems, except for chassis
        RobotContainer.robotVisualizer.updateRobotVisualization()

        // Return to non-RT thread priority (do not modify the first argument)
        // TODO(2): Same as TODO(1)
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        RobotContainer.robotDisabledConfig()
        RobotContainer.resetSimulation()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        RobotContainer.robotEnabledConfig()
        RobotContainer.robotAutonomousInitConfig()
        autonomousCommand = RobotContainer.getAutonomousCommand()

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand)
        }
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        RobotContainer.robotEnabledConfig()
        RobotContainer.robotTeleopInitConfig()
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        RobotContainer.robotEnabledConfig()
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {
        // Prevents the field bumps from being treated as wall-obstacles, so that RobotBumpSim can work.
        SimulatedArena.overrideInstance(RobotContainer.simField)
    }

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        // IMPORTANT: This method must be called ONLY in simulated robots, otherwise the roboRIO will SUFFER
        RobotContainer.updateSimulation()
    }

    /** Merely records metadata, sets the [Logger] receriver / replay source and starts it. */
    private fun initLogging() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        // Set up data receivers & replay source
        when (RobotConstants.ROBOT_MODE) {
            REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            SIM ->         // Running a physics simulator, log to NT
                Logger.addDataReceiver(NT4Publisher())

            REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        // Start AdvantageKit logger
        Logger.start()

        // Due to Kotlin's lazy loading of objects, this needs to be referenced here for Boundary objects to call
        // their init {} block, which logs the boundary box. This line does nothing else than loading the object into memory.
        FieldConstants.BoundaryLimits
    }
}