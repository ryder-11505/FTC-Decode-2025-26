package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.CurrentCutoff
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState
import org.firstinspires.ftc.teamcode.messages.StringMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.subsystems.Intake.PARAMS

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField
        var speed = 1.0 // 1.0 power ≈ 5600 RPM ∴ RPM = 5600(power) or RPM/5600 = power

        var varSpeed = 0.0 // depends on how far the tag is from the ll

    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "out")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun shoot() {
//        var initialized = false
//        return Loggable("SHOOT_RUN", fun(p: TelemetryPacket): Boolean {
//            if (!initialized) {
//                Logging.LOG("SHOOT")
                motor.power = speed
//                initialized = true
//            }
//
//            return false
//        })
    }

    fun setPower(RPM: Double) {
        val varSpeed = RPM / 5600

        motor.power = varSpeed
    }


    fun stopShoot() {
//        return Loggable("STOP_SHOOTER_MOTOR", InstantAction {
//            Logging.LOG("STOP_SHOOTER")
            motor.power = 0.0
//        })
    }

    override fun logState(uniqueName: String?) {
        Logging.DEBUG("$uniqueName INTAKE_POWER", motor.power)
        Logging.DEBUG("$uniqueName INTAKE_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
    }
}
