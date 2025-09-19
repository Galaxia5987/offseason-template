package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class WristPositions(val angle: Angle) {
    L1(10.0.degrees),
    L2(0.0.degrees),
    L3(135.0.degrees),
    L4(140.0.degrees)
}
