package frc.robot.subsystems.gripper

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.seconds
import frc.robot.subsystems.wrist.WristPositions
import frc.robot.wrist
import java.util.function.BooleanSupplier

fun outtakeBySensorWithWristPosition(): Command{
        return Commands.either(
            gripper.outtake({true}), gripper.outtake({false}), {wrist.setPoint == WristPositions.L3 || wrist.setPoint == WristPositions.L4})
            .andThen(Commands.waitTime(0.2.seconds))
            .andThen(gripper.stopIntakeOuttake())
}
