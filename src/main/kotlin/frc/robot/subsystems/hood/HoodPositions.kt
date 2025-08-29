package frc.robot.subsystems.hood

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class HoodPositions(val angle: Angle) {
    TAKEOUT(55.0.degrees),
    VERTICAL(90.0.degrees),
    UP(180.0.degrees),
    Down(0.0.degrees)
}
