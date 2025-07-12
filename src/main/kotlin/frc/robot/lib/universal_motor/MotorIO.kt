package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.volts
import org.team9432.annotation.Logged

interface MotorIO {
    val inputs: LoggedMotorInputs
    val config: TalonFXConfiguration

    fun setRequest(controlRequest: ControlRequest) {}

    fun updateInputs() {}

    @Logged
    open class MotorInputs {
        var position: Angle = 0.deg
        var distance: Distance = 0.m
        var voltage: Voltage = 0.volts
        var current: Current = 0.amps
    }
}
