package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun goToL1(): Command {
    return Commands.runOnce({ elevator.GoToL1().alongWith(wrist.moveToL1()) })
}

fun goToL2(): Command {
    return Commands.runOnce({ elevator.GoToL2().alongWith(wrist.moveToL2()) })
}

fun goToL3(): Command {
    return Commands.runOnce({ elevator.GoToL3().alongWith(wrist.moveToL3()) })
}

fun goToL4(): Command {
    return Commands.runOnce({ elevator.GoToL4().alongWith(wrist.moveToL4()) })
}
