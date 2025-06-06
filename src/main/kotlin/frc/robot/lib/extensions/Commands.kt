// Copyright FRC 1657
// Adapted from: https://github.com/Hamosad1657/HamosadLib/blob/main/src/main/kotlin/com/hamosad1657/lib/commands/Extentions.kt

package frc.robot.lib.extensions

import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier

infix fun Command.until(condition: BooleanSupplier): Command = until(condition)
infix fun Command.andThen(next: Command): Command = andThen(next)
infix fun Command.andThen(next: () -> Command): Command = andThen(next())
infix fun Command.finally(end: (interrupted: Boolean) -> Unit): Command = finallyDo(end)
infix fun Command.finally(command: Command): Command = finallyDo { _ -> command.schedule() }

infix fun Command.alongWith(parallel: Command): Command = alongWith(parallel)
infix fun Command.raceWith(parallel: Command): Command = raceWith(parallel)

infix fun Command.withTimeout(seconds: Double): Command = withTimeout(seconds)
infix fun Command.withTimeout(seconds: Time): Command = withTimeout(seconds)

/**
 * Automatically add the subsystem's name if this a single-subsystem command.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
infix fun Command.named(commandName: String): Command {
    val prefix = if (requirements.count() == 1) "${requirements.first().name}/" else ""
    return withName(prefix + commandName)
}

/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
fun withName(commandName: String, commandSupplier: () -> Command): Command =
    commandSupplier().also { it.name = commandName }
