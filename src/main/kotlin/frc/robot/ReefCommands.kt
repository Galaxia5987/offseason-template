package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.gripper.outtakeBySensorWithWristPosition

fun goToL1(): Command {
    return elevator.GoToL1().alongWith(wrist.moveToL1())//.andThen(outtakeBySensorWithWristPosition()))
}

fun goToL2(): Command {
    return elevator.GoToL2().alongWith(wrist.moveToL2())//.andThen(outtakeBySensorWithWristPosition())
}

fun goToL3(): Command {
    return elevator.GoToL3().alongWith(wrist.moveToL3())//.andThen(outtakeBySensorWithWristPosition())
}

fun goToL4(): Command {
    return elevator.GoToL4().alongWith(wrist.moveToL4())//.andThen(outtakeBySensorWithWristPosition())
}

fun intaking(): Command {
    return goToL1().alongWith(gripper.intake())
}

fun startOutTaking(): Command {
    return gripper.stop().alongWith(setOutTaking())
}

fun outTakeL1(): Command {
    return wrist.moveToL1().alongWith(outtakeBySensorWithWristPosition())
}

fun outTakeL2(): Command {
    return wrist.moveToL2().alongWith(outtakeBySensorWithWristPosition())
}

fun outTakeL3(): Command {
    return wrist.moveToL3().alongWith(outtakeBySensorWithWristPosition())
}

fun outTakeL4(): Command {
    return wrist.moveToL4().alongWith(outtakeBySensorWithWristPosition())
}
