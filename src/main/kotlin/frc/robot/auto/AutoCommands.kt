package frc.robot.auto
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.lib.extensions.seconds

val autoFactory: AutoFactory =
    AutoFactory(
        { drive.pose },
        { pose -> drive.resetOdometry(pose) },
        { sample: SwerveSample -> drive.followTrajectory(sample) },
        false,
        drive
    )



fun path_center(): Command {
    return Commands.sequence(
        autoFactory.resetOdometry("Path_center"),
        autoFactory.trajectoryCmd("Path_center")
    )
}

fun path_right(): Command {
    return autoFactory.trajectoryCmd("Path_right")
}

fun path_left(): Command {
    return autoFactory.trajectoryCmd("Path_left")
}

fun default(): Command {
    return Commands.sequence(
        Commands.waitTime(2.0.seconds),
        autoFactory.resetOdometry("default"),
        Commands.waitTime(2.0.seconds),
        autoFactory.trajectoryCmd("default")
    )
}
