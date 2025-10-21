package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get

class Hood : SubsystemBase() {
    val motorHood: TalonFX = TalonFX(3)
    val positionReq: PositionVoltage = PositionVoltage(0.0)
    var setPoint: Double = 0.0
    fun setAngle(angle: Angle) {
        setPoint = angle[degrees]
        motorHood.setControl(positionReq.withPosition(angle[degrees]))
    }

    override fun periodic() {
        super.periodic()
    }
}