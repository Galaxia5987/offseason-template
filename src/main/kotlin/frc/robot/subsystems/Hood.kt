package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Hood : SubsystemBase() {
    private val config =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().apply { //Motor's default actions
                    NeutralMode = NeutralModeValue.Brake //Motor Locked
                    Inverted = InvertedValue.CounterClockwise_Positive //Motor's positive angle is against clockwise
                }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimit = 60.0 //Motors's total max I
                StatorCurrentLimitEnable = true //Apply Limit
                SupplyCurrentLimit = 30.0 //Specific motor's max I
                SupplyCurrentLimitEnable = true //Apply Limit
            }
        }
    private val motor: UniversalTalonFX =
        UniversalTalonFX(port = 0, config = config, gearRatio = 1.0 / 5.0)
    private val positionRequest: PositionVoltage = PositionVoltage(0.0) //Motor's final position when angle = 0.0
    private var setPoint = 0.0 //Var to check when angle is right

    fun setAngle(angle: Angle) {
        setPoint = angle[degrees] //setPoint = angle (digit reference degrees)
        motor.setControl(positionRequest.withPosition(angle[degrees])) //Motor moves angle[degrees]
    }

    override fun periodic() {
        motor.updateInputs() //update motor's inputs
        Logger.processInputs("Hood", motor.inputs) //Updates inputs
        Logger.recordOutput("Hood/setpoint", setPoint) //Updates setPoint
    }
}