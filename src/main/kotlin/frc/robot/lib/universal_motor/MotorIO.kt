package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.deg
import org.team9432.annotation.Logged

interface MotorIO {
    val inputs: LoggedMotorInputs
    val config: TalonFXConfiguration

    fun setRequest(controlRequest: ControlRequest) {}

    fun resetInternalEncoder(angle: Angle = 0.deg) {}

    fun updateInputs() {}

    @Logged
    open class MotorInputs {
        var position: Angle = Degrees.zero()
        var distance: Distance = Meters.zero()
        var voltage: Voltage = Volts.zero()
        var current: Current = Amps.zero()
    }
}
