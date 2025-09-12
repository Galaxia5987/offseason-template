package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.centimeters
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.millimeters
import javax.swing.text.Position

const val GEAR_RATIO = (12.0 / 72.0) * 2
val SPORCKET_DIAMETERS: Distance = 36.4.millimeters

enum class Corallevels (val position: Distance)  {
    LEVEL0(0.5.m),
    LEVEL1(0.15.m),
    LEVEL2(0.28.m),
    LEVEL3(0.61.m),
    LEVEL4(1.m),
}



