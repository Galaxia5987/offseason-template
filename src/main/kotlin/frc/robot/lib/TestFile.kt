package frc.robot.lib

import kotlin.random.Random
import org.team5987.annotation.LoggedOutput

@LoggedOutput
val testValGetter: Int
    get() = Random.nextInt()

@LoggedOutput fun asdasd(): Int = Random.nextInt()
