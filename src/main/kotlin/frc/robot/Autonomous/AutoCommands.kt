package frc.robot.autonomous

import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import com.google.flatbuffers.Constants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.IS_RED
import frc.robot.drive
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.mirror
import frc.robot.lib.extensions.seconds

// Apply the generated speeds

val autoFactory: AutoFactory = AutoFactory(
    { drive.pose },
    { pose -> drive.resetOdometry(pose) },
    { sample: SwerveSample -> drive.followTrajectory(sample) },
    false,
    drive
)

fun path_center(): Command{

    return autoFactory.trajectoryCmd("Path_center")
}

fun path_right(): Command{
    return autoFactory.trajectoryCmd("Path_right")
}

fun path_left(): Command{
    //return Commands.run({drive.resetOdometry(Pose2d())})
    return autoFactory.trajectoryCmd("Path_left")
}

//fun default(): Command{
//    return autoFactory.trajectoryCmd("default")
//}

public fun default(): Command{
    return Commands.sequence(
        Commands.waitTime(2.0.seconds),
        autoFactory.resetOdometry("default"),
        Commands.waitTime(2.0.seconds),
        autoFactory.trajectoryCmd("default")
    )
}


