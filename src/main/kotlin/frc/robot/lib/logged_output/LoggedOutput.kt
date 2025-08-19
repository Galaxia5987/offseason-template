package frc.robot.lib.logged_output

@Target(AnnotationTarget.FIELD, AnnotationTarget.FUNCTION)
annotation class LoggedOutput(val key: String)
