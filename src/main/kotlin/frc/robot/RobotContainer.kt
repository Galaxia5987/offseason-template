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

        driverController.povLeft().whileTrue(flywheel.setVoltage(5.0.volts))
        driverController.povRight().whileTrue(flywheel.setVoltage((-5.0).volts))

        Trigger({driverController.rightY != 0.0}).whileTrue(wrist.addToPosition(driverController.rightY.degrees))
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {

    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
