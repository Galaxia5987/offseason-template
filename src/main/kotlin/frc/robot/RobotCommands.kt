package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.meters
import frc.robot.subsystems.elevator.Corallevels
import org.littletonrobotics.junction.AutoLogOutput


@AutoLogOutput  var state = Stats.IDEALING
    val isIntTaking = Trigger {state == Stats.INTAKING }
    val isOutTaking = Trigger {state == Stats.OUTTAKING }
    val isInL1= Trigger{elevator.setPoint== Corallevels.LEVEL1.position}
    val isInL2= Trigger{elevator.setPoint== Corallevels.LEVEL2.position}
    val isInL3= Trigger{elevator.setPoint== Corallevels.LEVEL3.position}
    val isInl4= Trigger{elevator.setPoint== Corallevels.LEVEL4.position}



    @AutoLogOutput val hasCoral= Trigger{ gripper.sensorDistance<0.1.meters}



    fun blindRobotCommands(){
        isIntTaking.apply {
            and(hasCoral).onTrue(startOutTaking())
            and(hasCoral).onFalse(intaking())
        }
        isOutTaking.apply {
            and(hasCoral).onTrue(startOutTaking())
            and(isInL1).onTrue(outTakeL1())
            and(isInL2).onTrue(outTakeL2())
            and(isInL3).onTrue(outTakeL3())
            and(isInl4).onTrue(outTakeL4())
        }
    }

     fun setRobotStat(newStats: Stats): Command{
        return Commands.runOnce({state= newStats})
    }
    fun setIntTaking(): Command{
        return setRobotStat(Stats.INTAKING)
    }
    fun setOutTaking(): Command{
       return setRobotStat(Stats.OUTTAKING)
    }



