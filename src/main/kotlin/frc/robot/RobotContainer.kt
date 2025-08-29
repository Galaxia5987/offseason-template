package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.volts
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput

object RobotContainer {

    private val driverController = CommandPS5Controller(0)


    init {
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }


    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {

    }

    private fun configureButtonBindings() {
        driverController.circle().onTrue(hood.getToVertical())
        driverController.square().onTrue(hood.getToOuttake())
        driverController.triangle().onTrue(hood.getUp())
        driverController.cross().onTrue(hood.getDown())
        driverController.povUp().whileTrue(hood.moveByController(10.degrees))
        driverController.povDown().whileTrue(hood.moveByController((-10).degrees))

        driverController.povLeft().onTrue(flywheel.setVoltage(0.3.volts))
        driverController.povRight().onTrue(flywheel.setVoltage((-0.3).volts))
        driverController.povLeft().onFalse(flywheel.setVoltage(0.0.volts))
        driverController.povRight().onFalse(flywheel.setVoltage(0.0.volts))


        Trigger({driverController.rightY < 0.0}).and(driverController.R3()).whileTrue(wrist.addToPosition(10.0.degrees))
        Trigger({driverController.rightY > 0.1}).and(driverController.R3()).whileTrue(wrist.addToPosition(-10.0.degrees))
        driverController.R3().whileFalse(wrist.addToPosition(0.0.degrees))

    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {

    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
