package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ContinuousServo
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoToggle
import org.firstinspires.ftc.teamcode.messages.ColorMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import java.util.concurrent.TimeUnit
import kotlin.math.max

@Config
class Intake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField // Forward intake power
        var P_Intake: Double = 15.0

        @JvmField // Speed while intaking, if it is flying past lower this number
        var speed = 1.0
    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "in")
    val motor2 = hardwareMap.get(DcMotorEx::class.java, "in2")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun intake() {
        motor.power = speed
        motor2.power = - speed
    }

    fun outake() {
        motor.power = - speed
        motor2.power = speed
    }

    fun stopIntake() {
        motor.power = 0.0
        motor2.power = 0.0
    }

    override fun logState(uniqueName: String?) {
        Logging.DEBUG("$uniqueName INTAKE_POWER", motor.power)
        Logging.DEBUG("$uniqueName INTAKE_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
        Logging.DEBUG("$uniqueName INTAKE_POWER", motor2.power)
        Logging.DEBUG("$uniqueName INTAKE_CURRENT", motor2.getCurrent(CurrentUnit.AMPS))
    }

}
