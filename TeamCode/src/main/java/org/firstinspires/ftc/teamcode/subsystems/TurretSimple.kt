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

        @JvmField // Speed while intaking, if it is flying past lower this number
        var speed = 1.0

        // REV HD Hex Motor = 28 CPR, 20:1 gear ratio
        private const val encoderCPR = 28.0
        private const val gearRatio = 20.0
        private const val ticksPerRev = encoderCPR * gearRatio
        @JvmField
        var ticksPerDegree: Double = ticksPerRev / 360.0  // ≈ 1.56 ticks/degree

        // Mechanical limits (adjust to your turret’s safe travel)
        @JvmField
        val SPIN_MIN = -(28.85 * ticksPerDegree)   // ≈ -45     // we want it to be - 90

        @JvmField
        val SPIN_MAX = (28.85 * ticksPerDegree)   // ≈ +45      // we want it to be +380

        @JvmField
        var P = 16.0

        @JvmField
        var I = 0.0

        @JvmField
        var D = 0.0
    }

    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")

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
        spinMotor.setVelocityPIDFCoefficients(P, I, D, 0.0)
        spinMotor.direction = DcMotorSimple.Direction.FORWARD
    }

    fun track(degrees: Double) {
        spinMotor.targetPosition = ((degrees * ticksPerDegree).toInt())
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.power = 1.0
        if (abs(((degrees * ticksPerDegree).toInt()) - ((degrees * ticksPerDegree).toInt())) < spinMotor.targetPositionTolerance && abs(((degrees * ticksPerDegree).toInt()) - spinCurrentPosition) < spinMotor.targetPositionTolerance && spinMotor.isBusy) {
            spinMotor.power = 0.0
        }
        if (IntRange(-2, 2).contains(((degrees * ticksPerDegree).toInt()) - spinCurrentPosition)){
            spinMotor.power = 0.0
        }
    }

    fun scan() {

    }

    fun resetEncoder () {
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
