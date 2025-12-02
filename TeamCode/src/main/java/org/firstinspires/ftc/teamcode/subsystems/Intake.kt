package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Time
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.staticData.Logging

@Config
class Intake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField // Forward intake power
        var P_Intake: Double = 15.0

        @JvmField // Speed while intaking, if it is flying past lower this number
        var speed = 0.85
    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "in")
    val motor2 = hardwareMap.get(DcMotorEx::class.java, "in2")

    // ball1 is the first ball to be intaked and closest to the shooter, ball2 is in the middle,
    // and ball3 is the most recently intaked and closest to the intake
    val ball1 = DigitalInput(hardwareMap, "ball1")
    val ball2 = DigitalInput(hardwareMap, "ball2")
    val ball3 = DigitalInput(hardwareMap, "ball3")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.setCurrentAlert(4.5, CurrentUnit.AMPS)
        motor2.setCurrentAlert(4.5, CurrentUnit.AMPS)
    }


    fun intake() {
        motor.power = - speed
        motor2.power = speed
    }

    fun outake() {
        motor.power = speed
        motor2.power = - speed
    }

    fun stopIntake() {
        motor.power = 0.0
        motor2.power = 0.0
    }


    fun isIntakeFull(): Boolean {
        if (ball1.triggered && ball2.triggered && ball3.triggered) {
            return true
        } else {
            return false
        }
    }

    fun isIntakeEmpty(): Boolean {
        if (!ball1.triggered && !ball2.triggered && !ball3.triggered) {
            return true
        } else {
            return false
        }
    }

    override fun logState(uniqueName: String?) {
        Logging.DEBUG("$uniqueName INTAKE_POWER", motor.power)
        Logging.DEBUG("$uniqueName INTAKE_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
        Logging.DEBUG("$uniqueName INTAKE2_POWER", motor2.power)
        Logging.DEBUG("$uniqueName INTAKE2_CURRENT", motor2.getCurrent(CurrentUnit.AMPS))
    }

}
