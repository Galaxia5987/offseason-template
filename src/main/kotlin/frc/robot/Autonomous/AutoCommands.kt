package frc.robot.autonomous

import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.bindRobotCommands
import frc.robot.drive
import frc.robot.lib.getPose2d
import frc.robot.subsystems.drive.DriveCommands

val autoFactory: AutoFactory = AutoFactory(
    { drive.pose },
    { pose -> drive.resetOdometry(pose) },
    { sample: SwerveSample -> drive.followTrajectory(sample) },
    true,
    drive
)

fun path_center(): Command{
    return autoFactory.trajectoryCmd("Path_center")
}


