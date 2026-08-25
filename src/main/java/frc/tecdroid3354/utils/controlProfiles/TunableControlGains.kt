package frc.tecdroid3354.utils.controlProfiles

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import frc.tecdroid3354.utils.devices.KrakenMotors

/**
 * Stores your [ControlGains] and creates a [LoggedTunableNumber] for each one, accessible through
 * "Tunables/${tabName}/${coefficientName}".
 *
 * **The exclusion of kF is intentional**, since it is not accepted in neither of
 * [Slot0Configs], [Slot1Configs] or [Slot2Configs]
 *
 * It also features auxiliary methods to check whether the coefficients had changed and to create
 * [Slot0Configs], [Slot1Configs], or [Slot2Configs] with the updated coefficients.
 */
class TunableControlGains(private val tabName: String,
                          private val kP: Double, private val kI: Double, private val kD: Double,
                          private val kS: Double, private val kV: Double, private val kA: Double, private val kG: Double
) {
    private val logged_kP: LoggedTunableNumber = LoggedTunableNumber("$tabName/kP", kP)
    private val logged_kI: LoggedTunableNumber = LoggedTunableNumber("$tabName/kI", kI)
    private val logged_kD: LoggedTunableNumber = LoggedTunableNumber("$tabName/kD", kD)
    private val logged_kS: LoggedTunableNumber = LoggedTunableNumber("$tabName/kS", kS)
    private val logged_kV: LoggedTunableNumber = LoggedTunableNumber("$tabName/kV", kV)
    private val logged_kA: LoggedTunableNumber = LoggedTunableNumber("$tabName/kA", kA)
    private val logged_kG: LoggedTunableNumber = LoggedTunableNumber("$tabName/kG", kG)

    // For easier interaction with KrakenMotors methods.
    private var controlGains: ControlGains = ControlGains(kP, kI, kD, kS, kV, kA, kG)

    /**
     * Updates the [controlGains] variable with the new values.
     */
    private fun updateControlGains() {
        controlGains = ControlGains(logged_kP.get(), logged_kI.get(), logged_kD.get(),
            logged_kS.get(), logged_kV.get(), logged_kA.get(), logged_kG.get())
    }

    /**
     * Checks if any of the PID, SVAG tunable had updated.
     */
    fun hadTunableUpdated(): Boolean {
        return (logged_kP.hasChanged(hashCode()) || logged_kI.hasChanged(hashCode()) || logged_kD.hasChanged(hashCode())
                || logged_kS.hasChanged(hashCode()) || logged_kV.hasChanged(hashCode())
                || logged_kA.hasChanged(hashCode()) || logged_kG.hasChanged(hashCode()))
    }

    /**
     * Updates the control gains through [updateControlGains] and returns a new [Slot0Configs]
     * through [KrakenMotors.configureSlot0]
     */
    fun updatePhoenixSlot0Configs(): Slot0Configs {
        updateControlGains()
        return KrakenMotors.configureSlot0(controlGains)
    }

    /**
     * Updates the control gains through [updateControlGains] and returns a new [Slot1Configs]
     * through [KrakenMotors.configureSlot1]
     */
    fun updatePhoenixSlot1Configs(): Slot1Configs {
        updateControlGains()
        return KrakenMotors.configureSlot1(controlGains)
    }

    /**
     * Updates the control gains through [updateControlGains] and returns a new [Slot2Configs]
     * through [KrakenMotors.configureSlot2]
     */
    fun updatePhoenixSlot2Configs(): Slot2Configs {
        updateControlGains()
        return KrakenMotors.configureSlot2(controlGains)
    }
}