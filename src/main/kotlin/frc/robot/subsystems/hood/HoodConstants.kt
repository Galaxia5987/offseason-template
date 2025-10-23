package frc.robot.subsystems.hood

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

val GEAR_RATIO = 4.0/56.0

public enum class Angles(val angle: Angle){
    ANGLE1(0.2.degrees),
    ANGLE2(0.4.degrees),
    ANGLE3(0.6.degrees),
    ANGLE4(2.0.degrees)
}

val hood = Hood()