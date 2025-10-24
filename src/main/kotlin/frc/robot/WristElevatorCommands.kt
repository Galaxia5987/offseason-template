package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorPositions
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristPositions

fun level0WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL0)
        .alongWith(Wrist.setAnglePosition(WristPositions.CLOSE))
}

fun level1WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL1)
        .alongWith(Wrist.setAnglePosition(WristPositions.OPEN))
}

fun level2WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL2)
        .alongWith(Wrist.setAnglePosition(WristPositions.OPEN))
}

fun level3WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL3)
        .alongWith(Wrist.setAnglePosition(WristPositions.OPEN))
}

fun level4WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL4)
        .alongWith(Wrist.setAnglePosition(WristPositions.OPEN))
}

fun level0WristElevatorInTake(): Command {
    return Gripper.inTakeUntilSenses()
        .alongWith(level0WristElevator())
}

fun level1OutTake(): Command {
    return Commands.sequence(level1WristElevator(), Commands.waitUntil(Wrist.atSetPoint.and(Elevator.atSetPoint)), Gripper.outTake())
}

fun level2OutTake(): Command {
    return Commands.sequence(level2WristElevator(), Commands.waitUntil(Wrist.atSetPoint.and(Elevator.atSetPoint)), Gripper.outTake())
}

fun level3OutTake(): Command {
    return Commands.sequence(level3WristElevator(), Commands.waitUntil(Wrist.atSetPoint.and(Elevator.atSetPoint)), Gripper.outTake())
}

fun level4OutTake(): Command {
    return Commands.sequence(level4WristElevator(), Commands.waitUntil(Wrist.atSetPoint.and(Elevator.atSetPoint)), Gripper.outTake())
}