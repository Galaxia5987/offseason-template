package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)
    public var isFinished12 = false
    public var isFinished34 = false

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
        driverController.cross().onTrue(elevator.GoToL3())
        driverController
            .square()
            .whileTrue(gripper.input())
        driverController
            .circle()
            .whileTrue(gripper.output())
        driverController
            .povUp()
            .whileTrue(elevator.GoToL4().andThen(wrist.moveToL4()))
        driverController
            .povDown()
            .whileTrue(elevator.GoToL1().andThen(wrist.moveToL1()))

        driverController
            .povLeft()
            .whileTrue(elevator.GoToL2().andThen(wrist.moveToL2()))

        driverController
            .povRight()
            .whileTrue(elevator.GoToL3().andThen(wrist.moveToL3()))


        //        // Lock to 0° when A button is held
        //        driverController
        //            .cross()
        //            .whileTrue(
        //                DriveCommands.joystickDriveAtAngle(
        //                    drive,
        //                    { -driverController.leftY },
        //                    { -driverController.leftX },
        //                    { Rotation2d() }
        //                )
        //            )
        //
        //        // Switch to X pattern when X button is pressed
        //        driverController.square().onTrue(runOnce(drive::stopWithX, drive))
        //
        //        // Reset gyro / odometry
        //        val resetOdometry =
        //            if (CURRENT_MODE == Mode.SIM)
        //                Runnable {
        //                    drive.resetOdometry(
        //                        driveSimulation!!.simulatedDriveTrainPose
        //                    )
        //                }
        //            else
        //                Runnable {
        //                    drive.resetOdometry(
        //                        Pose2d(drive.pose.translation, Rotation2d())
        //                    )
        //                }
        //        driverController
        //            .options()
        //            .onTrue(runOnce(resetOdometry).ignoringDisable(true))
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
