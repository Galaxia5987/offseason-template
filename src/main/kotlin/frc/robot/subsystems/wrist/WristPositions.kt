package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.degrees

enum class WristPositions(val angle: Angle) {
    OPEN(90.0.degrees),
    CLOSE(0.0.degrees);
}