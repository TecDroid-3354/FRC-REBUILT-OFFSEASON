package frc.tecdroid3354.subsystems.Flywheel

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.SubsystemTolerances
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsTunableValues
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import frc.tecdroid3354.utils.interpolation.InterpolatingDouble
import frc.tecdroid3354.utils.interpolation.InterpolatingTreeMap
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.rotationsPerMinute
import frc.tecdroid3354.utils.seconds
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.math.abs

class FlywheelSubsystem(private val io: FlywheelIO) : SubsystemBase(FlywheelConstants.Telemetry.SUBSYSTEM_TAB) {
    // Auto generated file (by @AutoLog annotation in IO Layer)
    private val inputs: FlywheelIOInputsAutoLogged = FlywheelIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorLeftInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorRightOneInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorRightTwoInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.kBoth)

    val scoringInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()
    val assistInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()


    /**
     * START OF CONNECTION ALERT VARIABLES. These alerts are published separately from other inputs.
     * This is to make sure all connection alerts are found in a shared folder.
     */
    private val leadMotorConnectionAlert: Alert =
        Alert(FlywheelConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    private val followerMotorConnectionAlert: Alert =
        Alert(FlywheelConstants.Telemetry.FOLLOWER_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    /**
     * END OF CONNECTION ALERT VARIABLES
     */
    /**
     * Interpolating Variables
     * */



    /**
     * Used for all sensors / actuators configuration.
     */
    init {
        io.initialMotorConfiguration()
    }

    /**
     * Updates and logs all inputs defined in [FlywheelIO.FlywheelIOInputs].
     * Updates connection alerts based on inputs.
     * Listens to updates through [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]
     */
    override fun periodic() {
        // IMPORTANT: This must be the first line in periodic() so that all other methods work with fresh data.
        io.updateFlywheelInputs(inputs, leadMotorInputs, followerMotorLeftInputs,
            followerMotorRightOneInputs, followerMotorRightTwoInputs,)

        // Logs every field to the specified directory. It can be seen live through Elastic & AdvantageScope.
        Logger.processInputs(FlywheelConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(FlywheelConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(FlywheelConstants.Telemetry.FOLLOWER_LEFT_MOTOR_INPUTS_TAB, followerMotorLeftInputs)
        Logger.processInputs(FlywheelConstants.Telemetry.FOLLOWER_RIGHT_MOTOR_ONE_INPUTS_TAB, followerMotorRightOneInputs)
        Logger.processInputs(FlywheelConstants.Telemetry.FOLLOWER_RIGHT_MOTOR_TWO_INPUTS_TAB, followerMotorRightTwoInputs)

        // Update motor alerts based on inputs.
        leadMotorConnectionAlert.set(leadMotorInputs.isConnected.not())
        followerMotorConnectionAlert.set(followerMotorLeftInputs.isConnected.not())

        // Check if ControlGains coefficients were changed live and update the motors.
        if (SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateFlywheelMotorsControlGains(0) // Updates Slot0 because is the primary set
        }

        // Check if the manual target RPMs were changed live and update the target.
        if (SubsystemsTunableValues.FLYWHEEL_MANUAL_RPM.hasChanged(hashCode())) {
            io.updateFlywheelManualVelocity(
                SubsystemsTunableValues.FLYWHEEL_MANUAL_RPM.get().rotationsPerMinute)
        }
    }

    /**
     * Enables live-tuned velocity. See implementation comment for details.
     */
    fun enableFlywheelManualVelocity(): Runnable {
        return io.enableFlywheelManualVelocity()
    }

    /**
     * Enables pre-stored velocity. See implementation comment for details.
     */
    fun enableFlywheelPresetVelocity(): Runnable {
        return io.enableFlywheelPresetVelocity()
    }

    /**
     * Calls [getCalculatedFlywheelScoringVelocity], which is then fed to the I/O.
     *
     * See I/O implementation comment for details.
     * @param flywheelDistanceToTarget Differs from robot distance to target (odometry); account for offsets from robot center.
     */
    fun enableFlywheelCalculatedScoringVelocity(flywheelDistanceToTarget: Supplier<Distance>): Command {
        // The .run() is because of how Runnables work, it ensures it runs again and doesn't freeze (even inside a RunCommand)
        return RunCommand({
            val flywheelCalculatedVelocity = getCalculatedFlywheelScoringVelocity(flywheelDistanceToTarget.get())
            io.enableFlywheelCalculatedVelocity(flywheelCalculatedVelocity).run()
        }, this)
    }

    /**
     * Calls [getCalculatedFlywheelAssistVelocity], which is then fed to the I/O.
     *
     * See I/O implementation comment for details.
     * @param flywheelDistanceToTarget Differs from robot distance to target (odometry); account for offsets from robot center.
     */
    fun enableFlywheelCalculatedAssistVelocity(flywheelDistanceToTarget: Supplier<Distance>): Command {
        // The .run() is because of how Runnables work, it ensures it runs again and doesn't freeze (even inside a RunCommand)
        return RunCommand({
            val flywheelCalculatedVelocity = getCalculatedFlywheelAssistVelocity(flywheelDistanceToTarget.get())
            io.enableFlywheelCalculatedVelocity(flywheelCalculatedVelocity).run()
        }, this)
    }

    /**
     * Stops the flywheel. See implementation for details.
     */
    fun stopFlywheel(): Runnable {
        return io.stopFlywheel()
    }

    /** Returns the flywheel velocity according to the [inputs] */
    fun getFlywheelVelocity(): AngularVelocity {
        return inputs.flywheelActualVelocity
    }

    /** Returns the flywheel target velocity according to the [inputs] */
    fun getFlywheelTargetVelocity(): AngularVelocity {
        return inputs.flywheelTargetVelocity
    }

    /** Returns true when the flywheel is within tolerance for its target */
    fun getIsAtTarget(): Boolean {
        val currentTarget = getFlywheelTargetVelocity().rotationsPerMinute
        val currentVelocity = getFlywheelVelocity().rotationsPerMinute

        if (currentTarget < 200.0) return false // The first loop might be currentTarget = 0.0, which triggers true
        val isWithinTolerance = abs(currentTarget - currentVelocity) <
                if (RobotConstants.IS_ROBOT_SIM) SubsystemTolerances.FLYWHEEL_SIM_TARGET_TOLERANCE.rotationsPerMinute else
                    SubsystemTolerances.FLYWHEEL_TARGET_TOLERANCE.rotationsPerMinute

        return atTargetDebouncer.calculate(isWithinTolerance) // Don't let it switch true / false constantly
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Coast.
     */
    fun coastFlywheelMotors(): Command {
        return io.coastFlywheelMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Brake.
     */
    fun brakeFlywheelMotors(): Command {
        return io.brakeFlywheelMotors().InstantCommandIgnoreDisabled(this)
    }

    fun getCalculatedScoringTimeOfFlight(flywheelDistanceToTarget: Distance): Time {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedTOF = FlywheelConstants.StoredPolynomials.TOF_SCORING_POLYNOMIAL.getOrThrow().evaluate(distanceInMeters)

        return calculatedTOF.seconds
    }

    fun getCalculatedAssistTimeOfFlight(flywheelDistanceToTarget: Distance): Time {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedTOF = FlywheelConstants.StoredPolynomials.TOF_ASSIST_POLYNOMIAL.getOrThrow().evaluate(distanceInMeters)

        return calculatedTOF.seconds
    }

    /**
     * Uses the stored scoring polynomial in [FlywheelConstants.StoredPolynomials] and evaluates
     * with the given flywheel distance to target.
     *
     *
     * Assumed Units:
     *
     *   - Distance: Meters
     *
     *   - Polynomial Output: Rotations Per Minute
     *
     * @param flywheelDistanceToTarget Differs from robot distance to target; account for offsets from robot center.
     * @return The [AngularVelocity] calculated by the scoring polynomial.
     */
    private fun getCalculatedFlywheelScoringVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters

        var calculatedRPMs = when(RobotConstants.USING_INTERPOLATION){
            true -> scoringInterpolation.getInterpolated(InterpolatingDouble(distanceInMeters)).value
            false -> FlywheelConstants.StoredPolynomials.SCORING_POLYNOMIAL.getOrThrow().evaluate(distanceInMeters)
        }

        return calculatedRPMs.rotationsPerMinute
    }

    /**
     * Uses the stored assist polynomial in [FlywheelConstants.StoredPolynomials] and evaluates
     * with the given flywheel distance to target.
     *
     *
     * Assumed Units:
     *
     *   - Distance: Meters
     *
     *   - Polynomial Output: Rotations Per Minute
     *
     * @param flywheelDistanceToTarget Differs from robot distance to target; account for offsets from robot center.
     * @return The [AngularVelocity] calculated by the assist polynomial.
     */
    private fun getCalculatedFlywheelAssistVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        var calculatedRPMs = when(RobotConstants.USING_INTERPOLATION){
            true -> assistInterpolation.getInterpolated(InterpolatingDouble(distanceInMeters)).value
            false -> FlywheelConstants.StoredPolynomials.ASSIST_POLYNOMIAL.getOrThrow().evaluate(distanceInMeters)
        }

        return calculatedRPMs.rotationsPerMinute
    }

    private fun configureInterpolation() {
        for (points in FlywheelConstants.Interpolation.scoringInterpolation ) {
            scoringInterpolation.put(InterpolatingDouble(points.key.meters), InterpolatingDouble(points.value.rotationsPerMinute))
        }
        for (points in FlywheelConstants.Interpolation.assistInterpolation){
            assistInterpolation.put(InterpolatingDouble(points.key.meters), InterpolatingDouble(points.value.rotationsPerMinute))
        }
    }
}