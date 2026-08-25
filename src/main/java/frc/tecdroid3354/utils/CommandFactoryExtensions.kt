@file:Suppress("FunctionName", "unused")

package frc.tecdroid3354.utils

import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.tecdroid3354.utils.InstantCommand

/**
 * Just creates an [InstantCommand] based of the [Runnable] and with the desired [Subsystem] requirements.
 * @param requirements The [Subsystem]s necessary to perform the [Runnable]
 * @return an [InstantCommand] with the above specifications.
 */
fun Runnable.InstantCommand(vararg requirements: Subsystem): InstantCommand {
    return InstantCommand(this, *requirements)
}
/**
 * Just creates an [InstantCommand] based of the [Runnable] with .ignoringDisable(true)
 *   and with the desired [Subsystem] requirements.
 * @param requirements The [Subsystem]s necessary to perform the [Runnable]
 * @return an [InstantCommand] with the above specifications.
 */
fun Runnable.InstantCommandIgnoreDisabled(vararg requirements: Subsystem): WrapperCommand {
    return InstantCommand(this, *requirements).ignoringDisable(true)
}
/**
 * Just creates a [RunCommand] based of the [Runnable] and with the desired [Subsystem] requirements.
 * @param requirements The [Subsystem]s necessary to perform the [Runnable]
 * @return a [RunCommand] with the above specifications.
 */
fun Runnable.RunCommand(vararg requirements: Subsystem): RunCommand {
    return RunCommand(this, *requirements)
}

/**
 * Performs a [WaitUntilCommand] with the specified condition, and then performs an [InstantCommand] with the
 * specified requirements
 * @param condition What must become true before scheduling the [InstantCommand]
 * @param requirements The [Subsystem]s necessary to perform the [Runnable]
 * @return a [SequentialCommandGroup] with the above specifications.
 */
fun Runnable.InstantCommandAfterCondition(condition: () -> Boolean, vararg requirements: Subsystem): SequentialCommandGroup {
    return WaitUntilCommand(condition).andThen(this.InstantCommand(*requirements))
}

/**
 * Performs a [WaitUntilCommand] with the specified time delay, and then performs an [InstantCommand] with the
 * specified requirements
 * @param time How long before scheduling the [InstantCommand]
 * @param requirements The [Subsystem]s necessary to perform the [Runnable]
 * @return a [SequentialCommandGroup] with the above specifications.
 */
fun Runnable.InstantCommandAfterTime(time: Time, vararg requirements: Subsystem): SequentialCommandGroup {
    return WaitUntilCommand(time.seconds).andThen(this.InstantCommand(*requirements))
}
