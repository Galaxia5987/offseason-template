package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.auto.default
import frc.robot.auto.path_center
import frc.robot.auto.path_left
import frc.robot.auto.path_right
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        bindRobotCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { driverController.leftY },
                { driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
    }

    private fun configureButtonBindings() {
        // reset swerve
        driverController
            .options()
            .onTrue(
                drive.runOnce { drive.resetGyro() }.ignoringDisable(true),
            )

        // Switch to X pattern when X button is pressed
        driverController.square().onTrue(runOnce(drive::stopWithX, drive))
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        val namedCommands: Map<String, Command> = mapOf()

        NamedCommands.registerCommands(namedCommands)

        autoChooser.addDefaultOption("path_center", path_center())
        autoChooser.addOption("path_default", default())

        autoChooser.addOption("path_right", path_right())

        autoChooser.addOption("path_left", path_left())

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
    }

    private fun bindRobotCommands() {}

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
