package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorPositions
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristPositions

fun level0WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL0).alongWith(Wrist.setAnglePosition(WristPositions.CLOSE))
}

fun level1WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL1).alongWith(Wrist.setAnglePosition(WristPositions.OPEN))

}

fun level2WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL2).alongWith(Wrist.setAnglePosition(WristPositions.OPEN))

}

fun level3WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL3).alongWith(Wrist.setAnglePosition(WristPositions.OPEN))

}

fun level4WristElevator(): Command {
    return Elevator.setLevel(ElevatorPositions.LEVEL4).alongWith(Wrist.setAnglePosition(WristPositions.OPEN))

}