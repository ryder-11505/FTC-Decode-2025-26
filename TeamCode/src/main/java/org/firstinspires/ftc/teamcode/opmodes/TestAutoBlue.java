package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOneBlue;
import com.userjhansen.automap.Maps.InsideOneRed;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOneBlue;
import com.userjhansen.automap.Maps.OutsideOneRed;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto")
@Config
public class TestAutoBlue extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;
    public static TrajectoryActionBuilder addParts(TrajectoryActionBuilder traj, AutoPart[] parts) {
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value+0.00001);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static TrajectoryActionBuilder addActionAfterFirst(TrajectoryActionBuilder traj, AutoPart[] parts, Action action) {
        boolean addedFirst = false;
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }

            if (!addedFirst) {
                addedFirst = true;
                traj = traj.afterTime(0.1, action);
            }
        }
        return traj;
    }

    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        TurretSimple spinSimple = new TurretSimple(hardwareMap);


        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Action initAction = new Loggable("INIT", new ParallelAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        spinSimple.resetEncoder();
                        return false;
                    }
                }

        ));

//        Run initialisation tasks
        visionDetection.limelight.start();
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }



        boolean innerPosition = false;
        boolean isRed = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.y) innerPosition = false;

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
                isRed = true;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;
                isRed = false;

            }

            Logging.LOG("ALLIANCE", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("POSITION", innerPosition ? "INNER" : "OUTER");

            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Map mapR = innerPosition ? new InsideOneRed() : new OutsideOneRed();

        Map mapB = innerPosition ? new InsideOneBlue() : new OutsideOneBlue();

        Map map = isRed ? mapR : mapB;

        driveBase.localizer.setPose(isRed ? mapR.getStartPosition() : mapB.getStartPosition());

        double Xr = -64.0;

        double Yr = 34.5;

        double rH = driveBase.localizer.getPose().heading.toDouble();

        double Xb = -64.0;

        double Yb = -34.5;

        double bH = driveBase.localizer.getPose().heading.toDouble();

        double Dr = Math.sqrt(((72.0 - Yr) * (72.0 - Yr)) + ((72.0 + Xr) * (72.0 + Xr)));

        double Db = Math.sqrt(((-72.0 - Yb) * (-72.0 - Yb)) + ((72.0 + Xb) * (72.0 + Xb)));

        double Hr = Math.atan((72.0 + Xr) / (72.0 - Yr)) + Math.toRadians(90.0) - rH;

        double Hb = Math.atan((72.0 + Xb) / (72.0 + Yb)) + Math.toRadians(180.0) - bH;

        double Hr2 = Math.toRadians(180.0);

        double Hb2 = Math.toRadians(180.0);

        double RSr = (-0.032) * Dr * Dr + 11.68 * Dr + 386.0;

        double RSb = (-0.032) * Db * Db + 11.68 * Db + 386.0;

        double HAr = 10.0;

        double HAb = 10.0;

        double VSr = 1000.0;

        double VSb = 1000.0;

        TrajectoryActionBuilder blueTraj = driveBase.actionBuilder(mapB.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(-64.0, -34.5))
//                .stopAndAdd(() -> {
//
//                    outtake.setPower(VSb);
//                    spinSimple.track(Math.toDegrees(Hb2));
//                    spinSimple.hoodAngle(HAb);
//
//                }).waitSeconds(1.5)
//                .stopAndAdd(() -> {
//
//                    intake.intake();
//                    outtake.open();
//
//                }).waitSeconds(2.5)
//                .stopAndAdd(() -> {
//
//                    intake.stopIntake();
//                    outtake.stopShoot();
//                    spinSimple.track(0.0);
//                    spinSimple.hoodAngle(10.0);
//
//                })
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-11.75, -27.0))
//                .stopAndAdd(() -> {
//
//                    intake.intake();
//
//                })
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-11.75, -50.0))
//                .stopAndAdd(() -> {
//
//                    intake.stopIntake();
//                    spinSimple.track(0.0);
//
//                })
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-64.0, -34.5))
                .endTrajectory();


        TrajectoryActionBuilder redTraj = driveBase.actionBuilder(mapR.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(-64.0, 34.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr);
                    spinSimple.track(Math.toDegrees(Hr2));
                    spinSimple.hoodAngle(HAr);

                }).waitSeconds(1.5)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0);
                    spinSimple.hoodAngle(10.0);

                })
                .strafeToConstantHeading(new Vector2d(-11.45, 27.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .strafeToConstantHeading(new Vector2d(-11.45, 50.0))
                .stopAndAdd(() -> {

                    intake.stopIntake();

                })
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-64.0, 34.5))
                .endTrajectory();


        TrajectoryActionBuilder blueTraj2 = driveBase.actionBuilder(mapB.getStartPosition()) // put initial pose here
                .strafeToLinearHeading(new Vector2d(-64.0, -34.5), new Rotation2d(Math.toRadians(0.0), Math.toRadians(270.0)))
                .stopAndAdd(() -> {

                    outtake.setPower(VSb);
                    spinSimple.track(Math.toDegrees(Hb2));
                    spinSimple.hoodAngle(HAb);

                }).waitSeconds(1.5)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0);
                    spinSimple.hoodAngle(10.0);

                })
                .strafeToConstantHeading(new Vector2d(-11.45, -27.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .strafeToConstantHeading(new Vector2d(-11.45, -50.0))
                .stopAndAdd(() -> {

                    intake.stopIntake();

                })
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-64.0, -34.5))
                .endTrajectory();

        TrajectoryActionBuilder redTraj2 = driveBase.actionBuilder(mapR.getStartPosition()) // put initial pose here
                .strafeToLinearHeading(new Vector2d(-64.0, 34.5), new Rotation2d(Math.toRadians(0.0), Math.toRadians(90.0)))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr);
                    spinSimple.track(Math.toDegrees(Hr2));
                    spinSimple.hoodAngle(HAr);

                }).waitSeconds(1.5)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0);
                    spinSimple.hoodAngle(10.0);

                })
                .strafeToConstantHeading(new Vector2d(-11.45, 27.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .strafeToConstantHeading(new Vector2d(-11.45, 50.0))
                .stopAndAdd(() -> {

                    intake.stopIntake();

                })
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-64.0, 34.5))
                .endTrajectory();



        waitForStart();

        Action traj1 = (isRed ? (innerPosition ? redTraj2.build() : redTraj.build()) : (innerPosition ? blueTraj2.build() : blueTraj.build()));

        Action traj2 = (isRed ? (innerPosition ? redTraj2.build() : redTraj.build()) : (innerPosition ? blueTraj2.build() : blueTraj.build()));

        Action intek = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.intake();
                return false;
            }
        };

        Actions.runBlocking(
                new SequentialAction(
                        traj1
                )
        );


        while (opModeIsActive() && !isStopRequested() && traj1.run(p)) {
            p = new TelemetryPacket();
            driveBase.update(p);
            PoseStorage.currentPose = driveBase.localizer.getPose();

            Logging.LOG("Red distance (IN)", Dr);
            Logging.LOG("X coordinate (IN)", driveBase.localizer.getPose().position.x);
            Logging.LOG("Y coordinate (IN)", driveBase.localizer.getPose().position.y);
            Logging.LOG("Heading angle (DEGREES)", Math.toDegrees(driveBase.localizer.getPose().heading.toDouble()));

            Logging.update();
        }
    }
}