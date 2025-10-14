package frc.robot

import edu.wpi.first.wpilibj2.command.Command

fun goToL1(): Command {
    return elevator.GoToL1().alongWith(wrist.moveToL1())
}

fun goToL2(): Command {
    return elevator.GoToL2().alongWith(wrist.moveToL2())
}

fun goToL3(): Command {
    return elevator.GoToL3().alongWith(wrist.moveToL3())
}

fun goToL4(): Command {
    return elevator.GoToL4().alongWith(wrist.moveToL4())
}

fun intaking(): Command {
    return goToL1().alongWith(gripper.input())
}

fun startOutTaking(): Command {
    return gripper.stop().alongWith(setOutTaking())
}

fun outTakeL1(): Command {
    return wrist.moveToL1().alongWith(gripper.output())
}

fun outTakeL2(): Command {
    return wrist.moveToL2().alongWith(gripper.output())
}

fun outTakeL3(): Command {
    return wrist.moveToL3().alongWith(gripper.output())
}

fun outTakeL4(): Command {
    return wrist.moveToL4().alongWith(gripper.output())
}
