package frc.tecdroid3354.subsystems.angularVelocity

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsTunableTargets
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import frc.tecdroid3354.utils.meters
import frc.tecdroid3354.utils.rotationsPerMinute
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

class FlywheelSubsystem(private val io: FlywheelIO) : SubsystemBase(FlywheelConstants.Telemetry.SUBSYSTEM_TAB) {
    // Auto generated file (by @AutoLog annotation in IO Layer)
    private val inputs: FlywheelIOInputsAutoLogged = FlywheelIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

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
        io.updateFlywheelInputs(inputs, leadMotorInputs, followerMotorInputs)

        // Logs every field to the specified directory. It can be seen live through Elastic & AdvantageScope.
        Logger.processInputs(FlywheelConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(FlywheelConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(FlywheelConstants.Telemetry.FOLLOWER_MOTOR_INPUTS_TAB, followerMotorInputs)

        // Update motor alerts based on inputs.
        leadMotorConnectionAlert.set(leadMotorInputs.isConnected.not())
        followerMotorConnectionAlert.set(followerMotorInputs.isConnected.not())

        // Check if ControlGains coefficients were changed live and update the motors.
        if (SubsystemsControlGains.FLYWHEEL_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateFlywheelMotorsControlGains(0) // Updates Slot0 because is the primary set
        }
        if (SubsystemsControlGains.FLYWHEEL_MOTOR_SECONDARY_GAINS.hadTunableUpdated()) {
            io.updateFlywheelMotorsControlGains(1) // Updates Slot1 because is the secondary set
        }

        // Check if the manual target RPMs were changed live and update the target.
        if (SubsystemsTunableTargets.FLYWHEEL_MANUAL_RPM.hasChanged(hashCode())) {
            io.updateFlywheelManualVelocity(
                SubsystemsTunableTargets.FLYWHEEL_MANUAL_RPM.get().rotationsPerMinute)
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
    fun enableFlywheelCalculatedScoringVelocity(flywheelDistanceToTarget: Distance): Runnable {
        val flywheelCalculatedVelocity = getCalculatedFlywheelScoringVelocity(flywheelDistanceToTarget)

        return io.enableFlywheelCalculatedVelocity(flywheelCalculatedVelocity)
    }

    /**
     * Calls [getCalculatedFlywheelAssistVelocity], which is then fed to the I/O.
     *
     * See I/O implementation comment for details.
     * @param flywheelDistanceToTarget Differs from robot distance to target (odometry); account for offsets from robot center.
     */
    fun enableFlywheelCalculatedAssistVelocity(flywheelDistanceToTarget: Distance): Runnable {
        val flywheelCalculatedVelocity = getCalculatedFlywheelAssistVelocity(flywheelDistanceToTarget)

        return io.enableFlywheelCalculatedVelocity(flywheelCalculatedVelocity)
    }

    /**
     * Stops the flywheel. See implementation for details.
     */
    fun stopFlywheel(): Runnable {
        return io.stopFlywheel()
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

    /**
     * Only if applicable. This is implemented here because it does not change between hardware / simulation layers.
     *
     *
     * Uses the stored scoring coefficients in [FlywheelConstants.PolynomialCoefficients] and evaluates
     * with the given flywheel distance to target.
     *
     *
     * Assumed Units:
     *
     *   - Distance: Meters
     *
     *   - Polynomial Output: Rotations Per Minute
     *
     * The output is divided by 60 before creating the [AngularVelocity] object, which accepts Rotations Per Second.
     * @param flywheelDistanceToTarget Differs from robot distance to target; account for offsets from robot center.
     * @return The [AngularVelocity] calculated by the scoring polynomial.
     */
    private fun getCalculatedFlywheelScoringVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedRPMs =
            FlywheelConstants.PolynomialCoefficients.SCORING_X3_COEFF.times(distanceInMeters.pow(3.0)) +
                    FlywheelConstants.PolynomialCoefficients.SCORING_X2_COEFF.times(distanceInMeters.pow(2.0)) +
                    FlywheelConstants.PolynomialCoefficients.SCORING_X1_COEFF.times(distanceInMeters) +
                    FlywheelConstants.PolynomialCoefficients.SCORING_X0_COEFF

        return calculatedRPMs.rotationsPerMinute
    }

    /**
     * Only if applicable. This is implemented here because it does not change between hardware / simulation layers.
     *
     *
     * Uses the stored assist coefficients in [FlywheelConstants.PolynomialCoefficients] and evaluates
     * with the given flywheel distance to target.
     *
     *
     * Assumed Units:
     *
     *   - Distance: Meters
     *
     *   - Polynomial Output: Rotations Per Minute
     *
     * The output is divided by 60 before creating the [AngularVelocity] object, which accepts Rotations Per Second.
     * @param flywheelDistanceToTarget Differs from robot distance to target; account for offsets from robot center.
     * @return The [AngularVelocity] calculated by the assist polynomial.
     */
    private fun getCalculatedFlywheelAssistVelocity(flywheelDistanceToTarget: Distance): AngularVelocity {
        val distanceInMeters = flywheelDistanceToTarget.meters
        val calculatedRPMs =
            FlywheelConstants.PolynomialCoefficients.ASSIST_X3_COEFF.times(distanceInMeters.pow(3.0)) +
                    FlywheelConstants.PolynomialCoefficients.ASSIST_X2_COEFF.times(distanceInMeters.pow(2.0)) +
                    FlywheelConstants.PolynomialCoefficients.ASSIST_X1_COEFF.times(distanceInMeters) +
                    FlywheelConstants.PolynomialCoefficients.ASSIST_X0_COEFF

        return calculatedRPMs.rotationsPerMinute
    }
}