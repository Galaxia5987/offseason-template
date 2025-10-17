package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.millimeters

const val GEAR_RATIO = (12.0 / 72.0) * 2
val SPORCKET_DIAMETERS: Distance = 36.4.millimeters

enum class Corallevels(val position: Distance) {
    LEVEL0(0.01.m),
    LEVEL1(0.01.m),
    LEVEL2(0.10.m),
    LEVEL3(0.40.m),
    LEVEL4(0.95.m),
}
