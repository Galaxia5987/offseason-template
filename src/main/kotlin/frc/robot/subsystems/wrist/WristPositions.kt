package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class WristPositions(val angle: Angle) {
    L1(10.0.degrees),
    L2(0.degrees),
    L3(115.0.degrees),
    L4(125.0.degrees),
    FEEDER(18.0.degrees),
    DOWN(0.0.degrees)
}
