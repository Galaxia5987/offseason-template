package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command

/**
 * A command that runs a `startCommand` when initialized and runs an
 * `endCommand` when it is finished. Similar to the WPILib's `Commands.startEnd`
 * function, but requires two Commands instead of two lambdas.
 *
 * @param startCommand The command to be executed when this command is
 * initialized.
 * @param endCommand The command to be executed when this command ends.
 */
class StartEndCommand(
    private val startCommand: Command,
    private val endCommand: Command
) : Command() {
    init {
        addRequirements(startCommand.requirements + endCommand.requirements)
    }

    override fun initialize() {
        startCommand.initialize()
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            endCommand.initialize()
        }
    }
}
