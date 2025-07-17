package frc.robot.subsystems.arm

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Arm : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)

    private var setpoint: Angle = 0.deg

    private val positionRequest = PositionVoltage(0.0.deg)

    private fun setPosition(angle: Angle): Command =
        runOnce {
                setpoint = angle
                motor.setControl(positionRequest.withPosition(angle))
            }
            .withName("Arm/setPosition")

    private fun setPosition(pos: ArmPosition): Command = setPosition(pos.angle)

    fun setIntaking(): Command = setPosition(ArmPosition.INTAKING)
    fun setUp(): Command = setPosition(ArmPosition.UP)

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(this::class.simpleName, motor.inputs)
        Logger.recordOutput("Arm/Setpoint", setpoint)
    }
}
