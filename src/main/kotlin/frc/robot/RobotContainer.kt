package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.rotationsPerSecond
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.Hood
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorPositions
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristPositions
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = CommandXboxController(0)

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
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
    }

    private fun configureButtonBindings() {
        driverController.povDown().onTrue(Elevator.setLevel(ElevatorPositions.LEVEL1)) // (-_-)
        driverController.povLeft().onTrue(Elevator.setLevel(ElevatorPositions.LEVEL2)) // (~_~)
        driverController.povRight().onTrue(Elevator.setLevel(ElevatorPositions.LEVEL3)) // (0_0)
        driverController.povUp().onTrue(Elevator.setLevel(ElevatorPositions.LEVEL4)) // (*u*)

        driverController.y().onTrue(Wrist.setAnglePosition(WristPositions.OPEN))
        driverController.a().onTrue(Wrist.setAnglePosition(WristPositions.CLOSE))

        driverController.b().onTrue(Gripper.inTake())
        driverController.x().onTrue(Gripper.outTake())
        driverController.leftBumper().onTrue(Gripper.stop())



    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        val namedCommands: Map<String, Command> = mapOf()

        NamedCommands.registerCommands(namedCommands)

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

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
