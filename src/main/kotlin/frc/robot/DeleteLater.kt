package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.degreesPerSecond
import frc.robot.subsystems.flywheel.Flywheel

object Feeder{
    fun releaseToShooter(): Command {
        return Commands.runOnce({})
    }
}
fun fullSequence() : Command {
    return Commands.sequence(
        Robot.hood.steepestAngle(),
        Robot.flywheel.rampToVelocity(1000.degreesPerSecond),
        Feeder.releaseToShooter()
    )
}