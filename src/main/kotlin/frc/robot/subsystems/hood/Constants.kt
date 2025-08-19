package frc.robot.subsystems.hood

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

const val gearRatio: Double= 1.0/2.3
enum class Hoodangles (val angle: Angle){
    ZERO(degrees.of(0.0)),
    FIVE(degrees.of(5.0)),
    TEN(degrees.of(10.0)),
    TWENTY(degrees.of(20.0)),
}