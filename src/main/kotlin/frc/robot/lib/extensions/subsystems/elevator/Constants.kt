package frc.robot.lib.extensions.subsystems.elevator

import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.centimeters
import frc.robot.lib.extensions.millimeters
import javax.swing.text.Position

val GEAR_RATIO= 1.0
val SPORCKET_DIAMETERS: Distance = 36.4.millimeters

enum class Corallevels (val position: Distance)  {
    LEVEL1(0.0.millimeters),
    LEVEL2(5000.0.millimeters),
    LEVEL3(10000.0.millimeters),
    LEVEL4(15000.0.millimeters),
}
val kP=1.0
val kI=0.0
val kD=0.0

