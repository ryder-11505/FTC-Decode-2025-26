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
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
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
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin.Companion
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin.Companion.tolerance
import org.firstinspires.ftc.teamcode.messages.ColorMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max

@Config
class TurretSimple(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {

        @JvmField
        var ticksPerDegree: Double = 3.5

        private const val encoderCPR = 28.0
        private var gearRatio = (ticksPerDegree * 360.0) / encoderCPR // ≈ 41.2380952381

        @JvmField
        var P = 25.0

        @JvmField
        var I = 0.0

        @JvmField
        var D = 7.0

        @JvmField
        var F = 0.0
    }

    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")
    val servo = hardwareMap.get(Servo::class.java, "servo2")

    val spinCurrentPosition get() = spinMotor.currentPosition
    val spinTargetPos get() = spinMotor.targetPosition

    private fun initMotor() {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
        spinMotor.targetPositionTolerance = 2
    }

    init {
        initMotor()
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.setPositionPIDFCoefficients(P)
        spinMotor.setVelocityPIDFCoefficients(P, I, D, F)
        spinMotor.direction = DcMotorSimple.Direction.FORWARD
        servo.position = 0.50
    }

    fun track(degrees: Double, offset: Double) {
        val degreesWithOffset = degrees + offset
        var safeDegrees = degreesWithOffset
        if (degreesWithOffset > 180) {
            safeDegrees = degreesWithOffset - 360
        } else if (degreesWithOffset < -180) {
            safeDegrees = 360 - degreesWithOffset
        } else if (IntRange(-180, 180).contains(degrees.toInt())) {
            safeDegrees = degreesWithOffset
        }
        val targetPosition = ((safeDegrees * ticksPerDegree).toInt())
        spinMotor.targetPosition = targetPosition
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.power = 1.0
        if (abs(((safeDegrees * ticksPerDegree).toInt()) - ((safeDegrees * ticksPerDegree).toInt())) < spinMotor.targetPositionTolerance && abs(((safeDegrees * ticksPerDegree).toInt()) - spinCurrentPosition) < spinMotor.targetPositionTolerance) {
            spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            spinMotor.power = 0.0
        }
        if (IntRange(-1, 1).contains(((safeDegrees * ticksPerDegree).toInt()) - spinCurrentPosition)){
            spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            spinMotor.power = 0.0
        }
    }

    fun hoodAngle(angle: Double) {
        val position = 0.0333333 * angle - 0.333333
        servo.position = position
    }

    fun resetEncoder() {
        spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName SPIN_POSITION", spinMotor.currentPosition)
        Logging.DEBUG("$uniqueName SPIN_TARGET", spinMotor.targetPosition)
        Logging.DEBUG("$uniqueName SPIN_MODE", spinMotor.mode)
        Logging.DEBUG("$uniqueName SPIN_POWER", spinMotor.power)
        Logging.DEBUG("$uniqueName SPIN_CURRENT", spinMotor.getCurrent(CurrentUnit.AMPS))
    }

}
