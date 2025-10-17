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
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.Turret
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max

@Config
class TurretSpin @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
    val ticksPerDegree: Double = Turret.ticksPerDegree,
) : StateLoggable {
    companion object {
        @JvmField
        var tolerance = 0.5
    }

    @JvmField
    val spinMotor = hardwareMap.get(DcMotorEx::class.java, name)

    @JvmField
    var targetDistance: Double = 0.0

    // Mechanical limits (adjust to your turret’s safe travel)
    @JvmField
    val SPIN_MIN = -(28.85 * Turret.ticksPerDegree).toInt()   // ≈ -45     // we want it to be - 90
    @JvmField
    val SPIN_MAX = (28.85 * Turret.ticksPerDegree).toInt()   // ≈ +45      // we want it to be +380

    val currentPosition get() = spinMotor.currentPosition.toDouble()

    private fun initMotor() {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
        spinMotor.targetPositionTolerance = (tolerance * ticksPerDegree).toInt()
    }

    init {
        initMotor()
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.setPositionPIDFCoefficients(pValue)
        spinMotor.direction = direction
    }

    private val spinActionWriter = DownsampledWriter("LIFT_ACTION", 50_000_000)
    private val spinPoseWriter = DownsampledWriter("LIFT_POSITION", 50_000_000)

    fun resetPosition(resetCondition: Action): LoggableAction {
        spinActionWriter.write(StringMessage("$lastName RESET_POSITION_START"))
        return object : LoggableAction {
            var initialized = false
            var endpointTimeout = Deadline(2, TimeUnit.SECONDS)
            val resetAction = SequentialAction(
                resetCondition,
                InstantAction {
                    spinMotor.power = 0.0
                },
                SleepAction(0.1)
            )
            override val name: String
                get() = "RESET_POSITION"

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    spinMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    spinMotor.power = -1.0
                    endpointTimeout.reset()
                    initialized = true
                }

                if (endpointTimeout.hasExpired() || !resetAction.run(p)) {
                    spinActionWriter.write(StringMessage("$lastName POSITION_RESET"))
                    FlightRecorder.write(
                        "$lastName RESET_TIMEOUT",
                        BooleanMessage(endpointTimeout.hasExpired())
                    )
                    spinPoseWriter.write(DoubleMessage(0.0))

                    spinMotor.power = 0.0
                    spinMotor.mode = DcMotor.RunMode.RESET_ENCODERS
                    initMotor()

                    // If there were any ignored goto distance calls make them happy now
                    // However does not wait for it
                    gotoDistance(targetDistance).run(p)

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
                get() = "$lastName GOTO_DISTANCE_$targetDistance"
            var initialized = false
            var timeout: Deadline? = null

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    targetDistance = max(distance, 0.1)
                    spinActionWriter.write(StringMessage("$lastName GOTO_DISTANCE"))

                    timeout = Deadline(
                        (0.75 * abs(targetDistance - currentPosition)).toLong(),
                        TimeUnit.SECONDS
                    )

                    // Clamp the target position to stay within limits
                    val desiredPosition = (targetDistance * ticksPerDegree).toInt()
                    val clampedPosition = desiredPosition.coerceIn(SPIN_MIN, SPIN_MAX)

                    spinMotor.targetPosition = clampedPosition
                    if (!lockedOut) {
                        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        spinMotor.power = 1.0
                    }

                    initialized = true
                }

                spinPoseWriter.write(DoubleMessage(currentPosition))

                if (abs(currentPosition - targetDistance) > tolerance && timeout?.hasExpired() == false && !PoseStorage.shouldHallucinate) {
                    return true
                }

                if (abs(targetDistance - 0.0) < Companion.tolerance && abs(targetDistance - currentPosition) < Companion.tolerance) {
//                    Target is 0, and we're there, turn off the motor
                    spinMotor.power = 0.0
                    spinMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                }
                return false
            }
        }
    }


    fun interface DoubleProvider {
        fun run(): Double
    }

    fun interface IntProvider {
        fun run(): Int
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
                    initialized = true
                }

                spinPoseWriter.write(DoubleMessage(currentPosition))

                if (!lockedOut) {
                    val position = positionProvider.run()
                    Logging.LOG("HOLDING_POSITION", position)

                    // Convert to ticks and clamp to safe range
                    val desiredPosition = (position * ticksPerDegree).toInt()
                    val clampedPosition = desiredPosition.coerceIn(SPIN_MIN, SPIN_MAX)

                    spinMotor.targetPosition = clampedPosition
                }

                return spinMotor.targetPosition != 0
            }
        }
    }

    var lockedOut = true
    fun lockout() {
        spinMotor.power = 0.0
        spinMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lockedOut = true
    }

    fun unlock() {
        if (abs(targetDistance - 0.0) > tolerance) {
            spinMotor.power = 1.0
            spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        lockedOut = false
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

