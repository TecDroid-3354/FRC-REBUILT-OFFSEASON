package frc.tecdroid3354.subsystems.Tower

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.tecdroid3354.constants.SubsystemTolerances
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsPresetTargets
import frc.tecdroid3354.constants.SubsystemsTunableValues
import frc.tecdroid3354.utils.InstantCommandIgnoreDisabled
import frc.tecdroid3354.utils.amps
import frc.tecdroid3354.utils.interfaces.MotorIOInputsAutoLogged
import frc.tecdroid3354.utils.rotationsPerMinute
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class TowerSubsystem(private val io: TowerIO) : SubsystemBase(TowerConstants.Telemetry.SUBSYSTEM_TAB) {
    // Auto generated file (by @AutoLog annotation in IO Layer)
    private val inputs: TowerIOInputsAutoLogged = TowerIOInputsAutoLogged()
    private val leadMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()
    private val followerMotorInputs: MotorIOInputsAutoLogged = MotorIOInputsAutoLogged()

    private val atTargetDebouncer: Debouncer = Debouncer(0.06, Debouncer.DebounceType.kBoth)

    /**
     * START OF CONNECTION ALERT VARIABLES. These alerts are published separately from other inputs.
     * This is to make sure all connection alerts are found in a shared folder.
     */
    private val leadMotorConnectionAlert: Alert =
        Alert(TowerConstants.Telemetry.LEAD_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
    private val followerMotorConnectionAlert: Alert =
        Alert(TowerConstants.Telemetry.FOLLOWER_MOTOR_CONNECTION_ALERT_TAB, Alert.AlertType.kError)
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
     * Updates and logs all inputs defined in [TowerIO.TowerIOInputs].
     * Updates connection alerts based on inputs.
     * Listens to updates through [frc.tecdroid3354.utils.controlProfiles.LoggedTunableNumber]
     */
    override fun periodic() {
        // IMPORTANT: This must be the first line in periodic() so that all other methods work with fresh data.
        io.updateTowerInputs(inputs, leadMotorInputs, followerMotorInputs)

        // Logs every field to the specified directory. It can be seen live through Elastic & AdvantageScope.
        Logger.processInputs(TowerConstants.Telemetry.SUBSYSTEM_TAB, inputs)
        Logger.processInputs(TowerConstants.Telemetry.LEAD_MOTOR_INPUTS_TAB, leadMotorInputs)
        Logger.processInputs(TowerConstants.Telemetry.FOLLOWER_MOTOR_INPUTS_TAB, followerMotorInputs)

        // Update motor alerts based on inputs.
        leadMotorConnectionAlert.set(leadMotorInputs.isConnected.not())
        followerMotorConnectionAlert.set(followerMotorInputs.isConnected.not())

        // Check if ControlGains coefficients were changed live and update the motors.
        if (SubsystemsControlGains.TOWER_MOTOR_PRIMARY_GAINS.hadTunableUpdated()) {
            io.updateTowerMotorsControlGains(0) // Updates Slot0 because is the primary set
        }

        // Check if the manual target RPMs were changed live and update the target.
        if (SubsystemsTunableValues.TOWER_MANUAL_RPM.hasChanged(hashCode())) {
            io.updateTowerManualVelocity(
                SubsystemsTunableValues.TOWER_MANUAL_RPM.get().rotationsPerMinute)
        }
    }

    /**
     * Enables live-tuned velocity. See implementation comment for details.
     */
    fun enableTowerManualVelocity(): Runnable {
        return io.enableTowerManualVelocity()
    }

    /**
     * Enables pre-stored velocity. See implementation comment for details.
     */
    fun enableTowerPresetVelocity(): Runnable {
        return io.enableTowerPresetVelocity()
    }

    /**
     * Stops the subsystem. See implementation for details.
     */
    fun stopTower(): Runnable {
        return io.stopTower()
    }

    /** Compares idle torque current vs actual torque current. The difference is due to FUELS traveling the tower. */
    fun getLoadCurrent(): Current {
        if (getIsAtTarget()) return abs(TowerConstants.Mechanical.IDLE_TORQUE_CURRENT
            .minus(leadMotorInputs.torqueCurrent).amps).amps
        return 0.0.amps
    }

    fun getIsAtTarget(): Boolean {
        val currentTarget = SubsystemsPresetTargets.TOWER_PRESET_RPM.rotationsPerMinute
        val currentVelocity = inputs.towerActualVelocity.rotationsPerMinute

        if (currentTarget < 200.0) return false // The first loop might be currentTarget = 0.0, which triggers true
        val isWithinTolerance = abs(currentTarget - currentVelocity) < SubsystemTolerances.TOWER_TARGET_TOLERANCE.rotationsPerMinute

        return atTargetDebouncer.calculate(isWithinTolerance) // Don't let it switch true / false constantly
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Coast.
     */
    fun coastTowerMotors(): Command {
        return io.coastTowerMotors().InstantCommandIgnoreDisabled(this)
    }

    /**
     * Changes NeutralMode / IdleMode of the motors to Brake.
     */
    fun brakeTowerMotors(): Command {
        return io.brakeTowerMotors().InstantCommandIgnoreDisabled(this)
    }
}