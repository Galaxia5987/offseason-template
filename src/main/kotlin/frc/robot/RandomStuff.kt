package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Feeder {
    fun releaseToShooter(): Command {
        return Commands.runOnce({})
    }
}

fun fullSequence(): Command {
    return Commands.sequence(
        Robot.hood.steepestAngle(),
        Feeder.releaseToShooter()
    )
}
