package frc.robot.lib.motors

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim

open class SimMotor {

    val motorSim: DCMotorSim
    var controller: PIDController? = null
    var profiledController: ProfiledPIDController? = null

    private var lastTimestampSeconds = 0.0
    var voltageRequest: MotorSetpoint = MotorSetpoint.simpleVoltage(0.0)

    val conversionFactor: Double

    constructor(
        model: LinearSystem<N2, N1, N2>,
        motor: DCMotor,
        gearing: Double,
        conversionFactor: Double
    ) {
        motorSim = DCMotorSim(model, motor.withReduction(gearing))
        this.conversionFactor = conversionFactor / gearing
    }

    constructor(
        motor: DCMotor,
        jKgMetersSquared: Double,
        gearing: Double,
        conversionFactor: Double
    ) : this(
        LinearSystemId.createDCMotorSystem(motor, jKgMetersSquared, gearing),
        motor,
        gearing,
        conversionFactor
    )

    fun setController(controller: PIDController) {
        this.controller = controller
    }

    fun setProfiledController(profiledController: ProfiledPIDController) {
        this.profiledController = profiledController
    }

    open fun update(timestampSeconds: Double) {
        motorSim.setInputVoltage(voltageRequest())
        motorSim.update(timestampSeconds - lastTimestampSeconds)
        lastTimestampSeconds = timestampSeconds
    }

    fun interface MotorSetpoint : () -> Double {
        companion object {
            fun simpleVoltage(voltage: Double): MotorSetpoint {
                return MotorSetpoint { voltage }
            }
        }
    }
}
