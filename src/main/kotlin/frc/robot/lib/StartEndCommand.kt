package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command

class StartEndCommand(
    private val startCommand: Command,
    private val endCommand: Command
) : Command() {
    init {
        addRequirements(startCommand.requirements + endCommand.requirements)
    }

    override fun initialize() {
        startCommand.initialize()
        print("INIT CALLED!!!!")
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            endCommand.initialize()
            print("END CALLED!!!!")
        }
    }
}
