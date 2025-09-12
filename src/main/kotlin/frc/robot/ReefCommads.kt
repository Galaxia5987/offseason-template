package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.wrist.Wrist

class ReefCommads(){
    val wrist = Wrist()
    val elevator = Elevator()
    fun goToL1() : Command{
        return Commands.runOnce({frc.robot.elevator.GoToL1().alongWith(frc.robot.wrist.moveToL1())})
    }

    fun goToL2() : Command{
        return Commands.runOnce({ frc.robot.elevator.GoToL2().alongWith(frc.robot.wrist.moveToL2())})
    }

    fun goToL3() : Command{
        return Commands.runOnce({ frc.robot.elevator.GoToL3().alongWith(frc.robot.wrist.moveToL3())})
    }

    fun goToL4() : Command{
        return Commands.runOnce({ frc.robot.elevator.GoToL4().alongWith(frc.robot.wrist.moveToL4())})
    }
}