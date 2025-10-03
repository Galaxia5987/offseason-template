package frc.robot

import edu.wpi.first.wpilibj2.command.Command

fun goToL1(): Command {
    return  elevator.GoToL1().alongWith(wrist.moveToL1())
}

fun goToL2(): Command {
    return  elevator.GoToL2().alongWith(wrist.moveToL2())
}

fun goToL3(): Command {
    return elevator.GoToL3().alongWith(wrist.moveToL3())
}

fun goToL4(): Command {
    return elevator.GoToL4().alongWith(wrist.moveToL4())
}
