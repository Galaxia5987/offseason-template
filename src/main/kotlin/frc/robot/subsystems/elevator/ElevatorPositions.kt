package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.meters

enum class ElevatorPositions(val position: Distance) {
    LEVEL1(0.25.meters),
    LEVEL2(0.5.meters),
    LEVEL3(0.75.meters),
    LEVEL4(1.0.meters);
}