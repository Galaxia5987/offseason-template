package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.meters

enum class ElevatorPositions(val position: Distance) {
    LEVEL1(1.0.meters),
    LEVEL2(1.5.meters),
    LEVEL3(3.0.meters),
    LEVEL4(3.5.meters);
}