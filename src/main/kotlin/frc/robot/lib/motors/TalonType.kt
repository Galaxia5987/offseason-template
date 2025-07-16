package frc.robot.lib.motors

import edu.wpi.first.math.system.plant.DCMotor

enum class TalonType {
    FALCON,
    FALCON_FOC,
    KRAKEN,
    KRAKEN_FOC;

    companion object {
        fun getDCMotor(motorType: TalonType, numMotors: Int): DCMotor {
            return when (motorType) {
                FALCON -> DCMotor.getFalcon500(numMotors)
                FALCON_FOC -> DCMotor.getFalcon500Foc(numMotors)
                KRAKEN -> DCMotor.getKrakenX60(numMotors)
                KRAKEN_FOC -> DCMotor.getKrakenX60Foc(numMotors)
            }
        }
    }
}