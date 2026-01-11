package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpHarry extends LinearOpMode {
    public static double SlowmodeSpeed = 0.5;
    public static double SlowmodeTurning = 0.5;

    public static double TriggerMin = 0.01;

    public int targetId = 24; // default tag ID

    private LoggableAction scanningAction = null;
    private LoggableAction trackingAction = null;
    private boolean isScanning = false;



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        TurretSimple spinSimple = new TurretSimple(hardwareMap);

        Button fieldMode = new Button();
        Button slowMode = new Button();

        Pose2d pose = driveBase.localizer.getPose();

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;

        driveBase.localizer.driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        PoseStorage.splitControls = true;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            visionDetection.update(driveBase.localizer, p);
            Logging.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);

//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true; // Tag ID 24
                targetId = 24;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false; // Tag ID 20
                targetId = 20;
            }
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        LoggableAction finishingAction = new Loggable("INIT", new ParallelAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        spinSimple.track(0.0, 0.0);
                        return false;
                    }
                }

        ));


        FinishingState finishState = FinishingState.Outtake;

        double angle = 25.0;

        double offset = 0.0;

        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        Deadline transferTimer = new Deadline(500, TimeUnit.MILLISECONDS);
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            driveBase.update(p);


            double X = driveBase.localizer.getPose().position.x;

            double Y = driveBase.localizer.getPose().position.y;

            double H = driveBase.localizer.getPose().heading.toDouble();

            double Dr = Math.sqrt(((72.0 - Y) * (72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

            double Db = Math.sqrt(((-72.0 - Y) * (-72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

            double Hr = Math.atan((72.0 + X) / (72.0 - Y)) + Math.toRadians(90.0) - H;

            double Hb = Math.atan((72.0 + X) / (72.0 + Y)) + Math.toRadians(180.0) - H;

            double Hr2 = (- H) - Math.acos((72.0 + X) / Dr);

            double Hb2 = (- H) + Math.acos((72.0 + X) / Db);

            double HAr = 0.190808 * Dr + 6.7288;

            double HAb = 0.190808 * Dr + 6.7288;

//            double HAb = 0.190808 * Db + 6.7288;

//            double HAr = -0.00548148 * Dr * Dr + 1.30889 * Dr - 29.0963;
//
//            double HAb = -0.00548148 * Db * Db + 1.30889 * Db - 29.0963;

            double VSr = 0.0;

            double VSb = 0.0;


            if (Dr <= 55.0) {
                VSr = 1010.0;
            } else if (55.0 < Dr && Dr <= 75.0) {
                VSr = 1140.0;
            } else if (75.0 < Dr && Dr <= 105.0) {
                VSr = 1200.0;
            } else if (105.0 < Dr && Dr <= 120.0) {
                VSr = 1240.0;
            } else if (120.0 < Dr && Dr <= 130.0) {
                VSr = 1280.0;
            } else if (130.0 < Dr && Dr <= 140.0) {
                VSr = 1320.0;
            } else if (140.0 < Dr && Dr <= 160.0) {
                VSr = 1340.0;
            } else if (160.0 < Dr && Dr <= 171.0) {
                VSr = 1380.0;
            }

            if (Db <= 55.0) {
                VSb = 1030.0;
            } else if (55.0 < Db && Db <= 75.0) {
                VSb = 1140.0;
            } else if (75.0 < Db && Db <= 105.0) {
                VSb = 1200.0;
            } else if (105.0 < Db && Db <= 120.0) {
                VSb = 1240.0;
            } else if (120.0 < Db && Db <= 130.0) {
                VSb = 1280.0;
            } else if (130.0 < Db && Db <= 140.0) {
                VSb = 1320.0;
            } else if (140.0 < Db && Db <= 160.0) {
                VSb = 1340.0;
            } else if (160.0 < Db && Db <= 171.0) {
                VSb = 1380.0;
            }

//            if (Db <= 55.0) {
//                VSb = 1010.0;
//            } else if (55.0 < Db && Db <= 75.0) {
//                VSb = 1080.0;
//            } else if (75.0 < Db && Db <= 105.0) {
//                VSb = 1120.0;
//            } else if (105.0 < Db && Db <= 120.0) {
//                VSb = 1180.0;
//            } else if (120.0 < Db && Db <= 130.0) {
//                VSb = 1220.0;
//            } else if (130.0 < Db && Db <= 140.0) {
//                VSb = 1280.0;
//            } else if (140.0 < Db && Db <= 160.0) {
//                VSb = 1320.0;
//            } else if (160.0 < Db && Db <= 171.0) {
//                VSb = 1340.0;
//            }


            if (gamepad2.b && PoseStorage.isRedAlliance) {
                outtake.setPower(VSr);
                Logging.LOG("b");
            } else if (gamepad2.b && !PoseStorage.isRedAlliance) {
                outtake.setPower(VSb);
                Logging.LOG("b");
            }

            if (gamepad2.bWasReleased()) {
                outtake.stopShoot();
            }

            if (gamepad2.right_bumper && PoseStorage.isRedAlliance) {
                if (gamepad2.dpadLeftWasPressed()) {
                    offset += 2.0;
                } else if (gamepad2.dpadRightWasPressed()) {
                    offset -= 2.0;
                }
                spinSimple.track(Math.toDegrees(Hr2), offset);
                spinSimple.hoodAngle(HAr);
            } else if (gamepad2.right_bumper && !PoseStorage.isRedAlliance) {
                if (gamepad2.dpadLeftWasPressed()) {
                    offset += 2.0;
                } else if (gamepad2.dpadRightWasPressed()) {
                    offset -= 2.0;
                }
                spinSimple.track(Math.toDegrees(Hb2), offset);
                spinSimple.hoodAngle(HAb);
            }

            if (gamepad2.rightBumperWasReleased()) {
                spinSimple.track(0.0, 0.0);
                spinSimple.hoodAngle(10.0);
                offset = 0.0;
            }





//            if (gamepad2.dpadUpWasPressed()) {
//                spinSimple.hoodAngle(angle += 1.0);
//                Logging.LOG("servo up");
//            }

//            if (gamepad2.dpadUpWasReleased()) {
//                spinSimple.hoodAngle(25.0);
//                Logging.LOG("servo mid");
//            }

//            if (gamepad2.dpadDownWasPressed()) {
//                spinSimple.hoodAngle(angle -= 1.0);
//                Logging.LOG("servo down");
//            }

//            if (gamepad2.dpadDownWasReleased()) {
//                spinSimple.hoodAngle(25.0);
//                Logging.LOG("servo mid");
//            }



//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;


            if (gamepad1.right_bumper) {
                PoseStorage.isRedAlliance = true; // Tag ID 24
            } else if (gamepad1.left_bumper) {
                PoseStorage.isRedAlliance = false; // Tag ID 20
            }

//            slowMode.update(gamepad1.y);
//            fieldMode.update(gamepad1.x);


            driveBase.update(p);


            if (gamepad2.x) {
                intake.outake();
                outtake.runServoBackwards();
                Logging.LOG("x");
            }

            if (gamepad2.xWasReleased()) {
                intake.stopIntake();
                outtake.stopServo();
            }

            if (gamepad2.y) {
                outtake.runServoBackwards();
                Logging.LOG("y");
            }

            if (gamepad2.yWasReleased()) {
                outtake.stopServo();
            }

            if (gamepad1.a || gamepad2.a) {
                intake.intake();
                Logging.LOG("a");
            }

            if (gamepad1.aWasReleased() || gamepad2.aWasReleased()) {
                intake.stopIntake();
            }


            if (gamepad2.left_bumper) {
                outtake.open();
                Logging.LOG("left_bumper");
            }

            if (gamepad2.leftBumperWasReleased()) {
                outtake.close();
            }


            Pose2d poseEstimate = driveBase.localizer.getPose();
            input = input.times(slowMode.val ? SlowmodeSpeed : 1);

            if (fieldMode.val) {
                input = poseEstimate.heading.inverse().plus((PoseStorage.isRedAlliance ? 1 : -1) * Math.PI / 2).times(input);
            }

            PoseVelocity2d drivePower = new PoseVelocity2d(
                    input,
                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
            );

            Logging.DEBUG("X Input", input.x);
            Logging.DEBUG("Y Input", input.y);

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            driveBase.setDrivePowers(drivePower);


            Logging.LOG("FINISH_STATE", finishState);
            if (finishingAction != null) {
                Logging.DEBUG("FINISH_ACTION", finishingAction.getName());

                if (!finishingAction.run(p)) {
                    Logging.LOG("FINISH_ACTION_FINISHED");
                    finishingAction = null;
                    finishState = FinishingState.None;
                }
            }

            Logging.LOG("SCANNING_MODE", isScanning);
            if (isScanning) {
                Logging.LOG("SCAN_STATE", scanningAction != null ? "SCANNING" : "TRACKING");
            }
//            pinpoint.update();
//            pose = pinpoint.getPosition();
//            Logging.LOG("X coordinate (IN)T", pose.getX(DistanceUnit.INCH));
//            Logging.LOG("Y coordinate (IN)T", pose.getY(DistanceUnit.INCH));
//            Logging.LOG("Heading angle (DEGREES)T", pose.getHeading(AngleUnit.DEGREES));
            Logging.LOG("angle", angle);
            Logging.LOG("speed", VSb);
            Logging.LOG("timer", transferTimer.timeRemaining(TimeUnit.MILLISECONDS));
            Logging.LOG("servo angle", spinSimple.getServo().getPosition());
            Logging.LOG("Blue angle (deg)", Math.toDegrees(Hb2));
            Logging.LOG("Red angle (deg)", Math.toDegrees(Hr2));
            Logging.LOG("Blue distance (IN)", Db);
            Logging.LOG("Red distance (IN)", Dr);
            Logging.LOG("X coordinate (IN)", driveBase.localizer.getPose().position.x);
            Logging.LOG("Y coordinate (IN)", driveBase.localizer.getPose().position.y);
            Logging.LOG("Heading angle (DEGREES)", Math.toDegrees(driveBase.localizer.getPose().heading.toDouble()));
            Logging.LOG("shooter velocity (ticks/s)", outtake.getMotor().getVelocity());
            Logging.LOG("shooter power", outtake.getMotor().getPower());
            Logging.LOG("Spin Target", spinSimple.getSpinTargetPos());
            Logging.LOG("Spin Pos (deg)", spinSimple.getSpinCurrentPosition() / (144.0 / 45.0 ));
            Logging.LOG("Spin Pos (ticks)", spinSimple.getSpinCurrentPosition());
            Logging.LOG("Spin Status", spinSimple.getSpinMotor().isBusy());
            Logging.LOG("Spin Power", spinSimple.getSpinMotor().getPower());
            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
//            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);
//            Logging.LOG("LL_TX", ll.getTx());
//            Logging.LOG("LL_TY", ll.getTy());
//            if (ll.isTagDetected()) {
//                Logging.LOG("LL_DETECTED_ID", ll.getDetectedId());
//            }


            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");

            Logging.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    enum FinishingState {
        Intake,
        Outtake,
        None
    }

    enum Motif {
        PPG,
        PGP,
        GPP,
        None
    }
}
