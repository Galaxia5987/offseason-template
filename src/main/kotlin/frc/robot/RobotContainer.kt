package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.toUnit
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)

//    private val autoChooser: LoggedDashboardChooser<Command>

    init {
//        drive // Ensure Drive is initialized

//        autoChooser =
//            LoggedDashboardChooser(
//                "Auto Choices",
//                AutoBuilder.buildAutoChooser()
//            )
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
//        drive.defaultCommand =
//            DriveCommands.joystickDrive(
//                { -driverController.leftY },
//                { -driverController.leftX },
//                { -driverController.rightX * 0.8 }
//            )
    }

    private fun configureButtonBindings() {
        driverController.circle().onTrue(hood.getToVertical())
        driverController.square().onTrue(hood.getToOuttake())
        driverController.triangle().onTrue(hood.getUp())
        driverController.cross().onTrue(hood.getDown())
        if (driverController.rightY < 0.1 || driverController.rightY > -0.1) {
            hood.setPosition(driverController.rightY.degrees)
        }

    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
//        val namedCommands: Map<String, Command> = mapOf()
//
//        NamedCommands.registerCommands(namedCommands)
//
//        // Set up SysId routines
//        autoChooser.addOption(
//            "Drive Wheel Radius Characterization",
//            DriveCommands.wheelRadiusCharacterization()
//        )
//        autoChooser.addOption(
//            "Drive Simple FF Characterization",
//            DriveCommands.feedforwardCharacterization()
//        )
//        autoChooser.addOption(
//            "Drive SysId (Quasistatic Forward)",
//            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
//        )
//        autoChooser.addOption(
//            "Drive SysId (Quasistatic Reverse)",
//            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
//        )
//        autoChooser.addOption(
//            "Drive SysId (Dynamic Forward)",
//            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
//        )
//        autoChooser.addOption(
//            "Drive SysId (Dynamic Reverse)",
//            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
//        )
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

//        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
