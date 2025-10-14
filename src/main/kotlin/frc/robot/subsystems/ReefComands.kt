package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.elevator
import frc.robot.wrist

fun level1(): Command {
    return elevator.GoToL1().alongWith(wrist.moveToL1())
}

fun level2(): Command {
    return elevator.GoToL2().alongWith(wrist.moveToL2())
}

fun level3(): Command {
    return elevator.GoToL3().alongWith(wrist.moveToL3())
}

fun level4(): Command {
    return elevator.GoToL4().alongWith(wrist.moveToL4())
}
