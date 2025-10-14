package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class WristPositions(val angle: Angle) {
    L1(230.0.degrees),
    L2(210.degrees),
    L3(200.0.degrees),
    L4(230.0.degrees)
}
