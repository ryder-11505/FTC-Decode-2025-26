package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.messages.BooleanMessage
import org.firstinspires.ftc.teamcode.messages.DoubleMessage
import org.firstinspires.ftc.teamcode.messages.StringMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max

@Config
class TurretFunctions @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
) : StateLoggable {
    companion object {
        @JvmField
        var tolerance = 0.5
    }

    @JvmField
    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")
    val tiltMotor = hardwareMap.get(DcMotorEx::class.java, "tilt")

    @JvmField
    var spinTargetDistance: Double = 0.0
    var tiltTargetDistance: Double = 0.0

    val spinCurrentPosition get() = spinMotor.currentPosition
    val tiltCurrentPosition get() = tiltMotor.currentPosition

    private fun initMotor() {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
        spinMotor.targetPositionTolerance = (tolerance * 0.1).toInt()
        tiltMotor.targetPosition = 0
        tiltMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        tiltMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        tiltMotor.power = 0.1
        tiltMotor.targetPositionTolerance = (tolerance * 0.1).toInt()
    }

    init {
        initMotor()
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.setPositionPIDFCoefficients(pValue)
        spinMotor.direction = direction
        tiltMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        tiltMotor.setPositionPIDFCoefficients(pValue)
        tiltMotor.direction = direction
    }

    private val spinActionWriter = DownsampledWriter("SPIN_ACTION", 50_000_000)
    private val spinPoseWriter = DownsampledWriter("SPIN_POSITION", 50_000_000)
    private val tiltActionWriter = DownsampledWriter("TILT_ACTION", 50_000_000)
    private val tiltPoseWriter = DownsampledWriter("TILT_POSITION", 50_000_000)

    fun resetPosition(resetCondition: Action): LoggableAction {
        spinActionWriter.write(StringMessage("$lastName RESET_POSITION_START"))
        tiltActionWriter.write(StringMessage("$lastName RESET_POSITION_START"))
        return object : LoggableAction {
            var initialized = false
            var endpointTimeout = Deadline(2, TimeUnit.SECONDS)
            val resetAction = SequentialAction(
                resetCondition,
                InstantAction {
                    spinMotor.power = 0.0
                    tiltMotor.power = 0.0
                },
                SleepAction(0.1)
            )
            override val name: String
                get() = "RESET_POSITION"

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    spinMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    spinMotor.power = -1.0
                    tiltMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    tiltMotor.power = -1.0
                    endpointTimeout.reset()
                    initialized = true
                }

                if (endpointTimeout.hasExpired() || !resetAction.run(p)) {
                    spinActionWriter.write(StringMessage("$lastName POSITION_RESET"))
                    tiltActionWriter.write(StringMessage("$lastName POSITION_RESET"))
                    FlightRecorder.write(
                        "$lastName RESET_TIMEOUT",
                        BooleanMessage(endpointTimeout.hasExpired())
                    )
                    spinPoseWriter.write(DoubleMessage(0.0))
                    tiltPoseWriter.write(DoubleMessage(0.0))

                    spinMotor.power = 0.0
                    spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    tiltMotor.power = 0.0
                    tiltMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    initMotor()

                    // If there were any ignored goto distance calls make them happy now
                    // However does not wait for it
                    gotoDistance(spinTargetDistance).run(p)
                    gotoDistance(tiltTargetDistance).run(p)

                    return false
                }
                return true
            }
        }
    }

    @JvmOverloads
    fun gotoDistance(distance: Double, tolerance: Double = Companion.tolerance): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName GOTO_DISTANCE_$spinTargetDistance && $tiltTargetDistance"
            var initialized = false
            var timeout: Deadline? = null

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    spinTargetDistance = max(distance, 0.1)
                    tiltTargetDistance = max(distance, 0.1)
                    spinActionWriter.write(StringMessage("$lastName GOTO_DISTANCE"))
                    tiltActionWriter.write(StringMessage("$lastName GOTO_DISTANCE"))

                    spinMotor.targetPosition = spinTargetDistance.toInt()
                    tiltMotor.targetPosition = tiltTargetDistance.toInt()

                    initialized = true
                }

                spinPoseWriter.write(DoubleMessage(spinCurrentPosition.toDouble()))
                tiltPoseWriter.write(DoubleMessage(tiltCurrentPosition.toDouble()))


                return false
            }
        }
    }


    fun interface DoubleProvider {
        fun run(): Double
    }

    fun holdVariablePosition(positionProvider: DoubleProvider): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName HOLD_USER_DISTANCE"
            var initialized = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    spinActionWriter.write(StringMessage("$lastName HOLD_USER_DISTANCE"))
                    spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    spinMotor.power = 1.0
                    tiltActionWriter.write(StringMessage("$lastName HOLD_USER_DISTANCE"))
                    tiltMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    tiltMotor.power = 1.0
                    initialized = true
                }

                spinPoseWriter.write(DoubleMessage(spinCurrentPosition.toDouble()))
                tiltPoseWriter.write(DoubleMessage(tiltCurrentPosition.toDouble()))


                return spinMotor.targetPosition != 0 && tiltMotor.targetPosition != 0
            }
        }
    }


    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName SPIN_POSITION", spinMotor.currentPosition)
        Logging.DEBUG("$uniqueName SPIN_TARGET", spinMotor.targetPosition)
        Logging.DEBUG("$uniqueName SPIN_MODE", spinMotor.mode)
        Logging.DEBUG("$uniqueName SPIN_POWER", spinMotor.power)
        Logging.DEBUG("$uniqueName SPIN_CURRENT", spinMotor.getCurrent(CurrentUnit.AMPS))
        Logging.DEBUG("$uniqueName TILT_POSITION", tiltMotor.currentPosition)
        Logging.DEBUG("$uniqueName TILT_TARGET", tiltMotor.targetPosition)
        Logging.DEBUG("$uniqueName TILT_MODE", tiltMotor.mode)
        Logging.DEBUG("$uniqueName TILT_POWER", tiltMotor.power)
        Logging.DEBUG("$uniqueName TILT_CURRENT", tiltMotor.getCurrent(CurrentUnit.AMPS))
    }
}