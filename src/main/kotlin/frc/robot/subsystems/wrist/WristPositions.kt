package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.extensions.degrees

enum class WristPositions (angle: Angle) {
    L1 (0.0.degrees),
    L2 (30.0.degrees),
    L3 (60.0.degrees),
    L4 (90.0.degrees);

    val wristAngle = angle
}