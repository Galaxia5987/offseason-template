package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.extensions.degrees

enum class WristPositions (val angle: Angle) {
    L1 (30.0.degrees),
    L2 (0.0.degrees),
    L3 (50.0.degrees),
    L4 (100.0.degrees);
}