package frc.robot.lib.motors

import com.ctre.phoenix6.controls.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts
import frc.robot.lib.math.differential.Derivative
import frc.robot.lib.motors.SimMotor.MotorSetpoint

class TalonFXSim : SimMotor {
    private val acceleration = Derivative()

    constructor(
        model: LinearSystem<N2, N1, N2>,
        numMotors: Int,
        gearing: Double,
        conversionFactor: Double,
        motorType: TalonType
    ) : super(model, TalonType.getDCMotor(motorType, numMotors), gearing, conversionFactor)

    constructor(
        motor: DCMotor,
        gearing: Double,
        jKgMetersSquared: Double,
        conversionFactor: Double
    ) : super(motor, jKgMetersSquared, gearing, conversionFactor)

    constructor(
        numMotors: Int,
        gearing: Double,
        jKgMetersSquared: Double,
        conversionFactor: Double,
        talonType: TalonType
    ) : super(
        TalonType.getDCMotor(talonType, numMotors),
        jKgMetersSquared,
        gearing,
        conversionFactor
    )

    override fun update(timestampSeconds: Double) {
        super.update(timestampSeconds)
        acceleration.update(getVelocity().`in`(Units.RotationsPerSecond), timestampSeconds)
    }

    fun setControl(request: DutyCycleOut) {
        setControl(VoltageOut(request.Output * 12))
    }

    fun setControl(request: VoltageOut) {
        voltageRequest = MotorSetpoint.simpleVoltage(request.Output)
    }

    fun setControl(request: PositionDutyCycle) {
        setControl(
            PositionVoltage(request.Position).withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: PositionVoltage) {
        voltageRequest = MotorSetpoint {
            controller!!.calculate(getPosition(), request.Position) + request.FeedForward
        }
    }

    fun setControl(request: VelocityDutyCycle) {
        setControl(
            VelocityVoltage(request.Velocity).withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: VelocityVoltage) {
        voltageRequest = MotorSetpoint {
            controller!!.calculate(
                getVelocity().`in`(Units.RotationsPerSecond),
                request.Velocity
            ) + request.FeedForward
        }
    }

    fun setControl(request: MotionMagicDutyCycle) {
        setControl(
            MotionMagicVoltage(request.Position).withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: MotionMagicVoltage) {
        voltageRequest = MotorSetpoint {
            profiledController!!.calculate(getPosition(), request.Position) + request.FeedForward
        }
    }

    fun getVelocity(): AngularVelocity {
        return Units.Rotation.per(Units.Minutes)
            .of(motorSim.angularVelocityRPM)
            .times(conversionFactor)
    }

    fun getPosition(): Double {
        return motorSim.angularPositionRotations * conversionFactor
    }

    fun getAcceleration(): Double = acceleration.get()

    fun getAppliedCurrent(): Current = motorSim.currentDrawAmps.amps

    fun getAppliedVoltage(): Voltage = motorSim.inputVoltage.volts
}
