package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LoggedOutput

const val LOOP_TIME = 0.02 // [s]

@LoggedOutput
val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv("isReplay") == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }

const val ALT_ROBORIO_SERIAL = ""

@LoggedOutput
val IS_RED: Boolean
    get() =
        DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red

enum class Mode {
    REAL,
    SIM,
    REPLAY
}
