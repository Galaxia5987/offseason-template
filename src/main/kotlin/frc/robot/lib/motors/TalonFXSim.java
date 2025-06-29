package frc.robot.lib.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.compound.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.math.differential.Derivative;

public class TalonFXSim extends SimMotor {

    private final Derivative acceleration = new Derivative();

    public TalonFXSim(
            LinearSystem<N2, N1, N2> model,
            int numMotors,
            double gearing,
            double conversionFactor,
            TalonType motorType) {
        super(model, TalonType.getDCMotor(motorType, numMotors), gearing, conversionFactor);
    }

    public TalonFXSim(
            DCMotor motor, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(motor, jKgMetersSquared, gearing, conversionFactor);
    }

    public TalonFXSim(
            int numMotors,
            double gearing,
            double jKgMetersSquared,
            double conversionFactor,
            TalonType talonType) {
        super(
                TalonType.getDCMotor(talonType, numMotors),
                jKgMetersSquared,
                gearing,
                conversionFactor);
    }

    @Override
    public void update(double timestampSeconds) {
        super.update(timestampSeconds);

        acceleration.update(getVelocity().in(Units.RotationsPerSecond), timestampSeconds);
    }

    public void setControl(DutyCycleOut request) {
        setControl(new VoltageOut(request.Output * 12));
    }

    public void setControl(VoltageOut request) {
        voltageRequest = MotorSetpoint.simpleVoltage(request.Output);
    }

    public void setControl(PositionDutyCycle request) {
        setControl(new PositionVoltage(request.Position).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(PositionVoltage request) {
        voltageRequest =
                () -> controller.calculate(getPosition(), request.Position) + request.FeedForward;
    }

    public void setControl(VelocityDutyCycle request) {
        setControl(new VelocityVoltage(request.Velocity).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(VelocityVoltage request) {
        voltageRequest =
                () ->
                        controller.calculate(
                                        getVelocity().in(Units.RotationsPerSecond),
                                        request.Velocity)
                                + request.FeedForward;
    }

    public void setControl(MotionMagicDutyCycle request) {
        setControl(
                new MotionMagicVoltage(request.Position).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(MotionMagicVoltage request) {
        voltageRequest =
                () ->
                        profiledController.calculate(getPosition(), request.Position)
                                + request.FeedForward;
    }

    public void setControl(VelocityTorqueCurrentFOC request) {
        voltageRequest = () ->
                controller.calculate(
                        getVelocity().in(Units.RotationsPerSecond),
                        request.Velocity)
                        + (request.FeedForward * 12);
    }

    public void setControl(TorqueCurrentFOC request) {
        setControl(new VoltageOut(request.Output * 12));
    }

    public void setControl(PositionTorqueCurrentFOC request) {
        voltageRequest = () ->
                controller.calculate(getPosition(), request.Position)
                        + request.FeedForward * 12;
    }

    public void setControl(MotionMagicTorqueCurrentFOC request) {
        voltageRequest = () ->
                profiledController.calculate(getPosition(), request.Position)
                        + request.FeedForward * 12;
    }

    public void setControl(MotionMagicVelocityDutyCycle request) {
        voltageRequest = () ->
                profiledController.calculate(
                        getVelocity().in(Units.RotationsPerSecond), request.Velocity)
                        + request.FeedForward * 12;
    }

    public void setControl(MotionMagicVelocityTorqueCurrentFOC request) {
        voltageRequest = () ->
                profiledController.calculate(
                        getVelocity().in(Units.RotationsPerSecond), request.Velocity)
                        + request.FeedForward * 12;
    }

    public void setControl(ControlRequest request)
    {
        if (request instanceof DutyCycleOut reqDutyCycleOut)
            setControl(reqDutyCycleOut);
        if (request instanceof TorqueCurrentFOC reqTorqueCurrentFOC)
            setControl(reqTorqueCurrentFOC);
        if (request instanceof VoltageOut reqVoltageOut)
            setControl(reqVoltageOut);
        if (request instanceof PositionDutyCycle reqPositionDutyCycle)
            setControl(reqPositionDutyCycle);
        if (request instanceof PositionVoltage reqPositionVoltage)
            setControl(reqPositionVoltage);
        if (request instanceof PositionTorqueCurrentFOC reqPositionTorqueCurrentFOC)
            setControl(reqPositionTorqueCurrentFOC);
        if (request instanceof VelocityDutyCycle reqVelocityDutyCycle)
            setControl(reqVelocityDutyCycle);
        if (request instanceof VelocityVoltage reqVelocityVoltage)
            setControl(reqVelocityVoltage);
        if (request instanceof VelocityTorqueCurrentFOC reqVelocityTorqueCurrentFOC)
            setControl(reqVelocityTorqueCurrentFOC);
        if (request instanceof MotionMagicDutyCycle reqMotionMagicDutyCycle)
            setControl(reqMotionMagicDutyCycle);
        if (request instanceof MotionMagicVoltage reqMotionMagicVoltage)
            setControl(reqMotionMagicVoltage);
        if (request instanceof MotionMagicTorqueCurrentFOC reqMotionMagicTorqueCurrentFOC)
            setControl(reqMotionMagicTorqueCurrentFOC);
        if (request instanceof MotionMagicVelocityDutyCycle reqMotionMagicVelocityDutyCycle)
            setControl(reqMotionMagicVelocityDutyCycle);
        if (request instanceof MotionMagicVelocityTorqueCurrentFOC reqMotionMagicVelocityTorqueCurrentFOC)
            setControl(reqMotionMagicVelocityTorqueCurrentFOC);
        throw new IllegalArgumentException("Unsupported Control Request!");
    }

    public AngularVelocity getVelocity() {
        return Units.Rotation.per(Units.Minutes)
                .of(motorSim.getAngularVelocityRPM())
                .times(conversionFactor);
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations() * conversionFactor;
    }

    public double getAcceleration() {
        return acceleration.get();
    }

    public Current getAppliedCurrent() {
        return Units.Amps.of(motorSim.getCurrentDrawAmps());
    }

    public Voltage getAppliedVoltage() {
        return Units.Volts.of(motorSim.getInputVoltage());
    }
}
