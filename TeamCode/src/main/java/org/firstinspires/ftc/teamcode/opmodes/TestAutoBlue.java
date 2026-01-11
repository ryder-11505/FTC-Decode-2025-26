package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.Outtake.closed;
import static org.firstinspires.ftc.teamcode.subsystems.Outtake.open;

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
//        visionDetection.limelight.start();
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }



        boolean innerPosition = false;
        boolean isRed = false;
        boolean sixBall = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.y) innerPosition = false;

            if (gamepad2.a) sixBall = true;
            else if (gamepad2.y) sixBall = false;

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
                isRed = true;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;
                isRed = false;

            }

            Logging.LOG("ALLIANCE", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("POSITION", innerPosition ? "INNER" : "OUTER");
            Logging.LOG("sixBall", sixBall ? "yes" : "no");

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

        driveBase.localizer.setPose(map.getStartPosition());

        double Xr = -64.0;

        double Yr = -34.5;

        double rH = driveBase.localizer.getPose().heading.toDouble();

        double Xb = -64.0;

        double Yb = 34.5;

        double bH = driveBase.localizer.getPose().heading.toDouble();

        double Xr2 = 60.0;

        double Yr2 = -17.5;

        double rH2 = driveBase.localizer.getPose().heading.toDouble();

        double Xb2 = 60.0;

        double Yb2 = 17.5;

        double bH2 = driveBase.localizer.getPose().heading.toDouble();

        double Dr = Math.sqrt(((72.0 + Yr2) * (72.0 + Yr2)) + ((72.0 + Xr2) * (72.0 + Xr2)));

        double Db = Math.sqrt(((-72.0 + Yb2) * (-72.0 + Yb2)) + ((72.0 + Xb2) * (72.0 + Xb2)));

        double Hr = Math.atan((72.0 + Xr) / (72.0 - Yr)) + Math.toRadians(90.0) - rH;

        double Hb = Math.atan((72.0 + Xb) / (72.0 + Yb)) + Math.toRadians(180.0) - bH;

        double Hr2 = 180.0;

        double Hb2 = 180.0;

        double HAr = 10.0;

        double HAb = 10.0;

        double VSr = 1050.0;

        double VSb = 1050.0;

        double Hr22 = -113.0;

        double Hb22 = 113.0;

        double HAr2 = 0.190808 * Dr + 6.7288;

        double HAb2 = 0.190808 * Dr + 6.7288;

        double VSr2 = 1460.0;

        double VSb2 = 1460.0;

        TrajectoryActionBuilder blueTraj = driveBase.actionBuilder(mapB.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(-64.0, 33.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSb);
                    spinSimple.track(Hb2, 0.0);
                    spinSimple.hoodAngle(HAb);

                }).waitSeconds(2.75)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(-11.45, 22.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-11.45, 47.0))
                .waitSeconds(1.0)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    spinSimple.track(0.0, 0.0);

                })
                .waitSeconds(1.0)
//                .splineToConstantHeading(new Vector2d(-64.0, 33.5), Math.toRadians(270.0))
                .strafeToConstantHeading(new Vector2d(-64.0, 33.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSb);
                    spinSimple.track(Hb2, 0.0);
                    spinSimple.hoodAngle(HAb);

                }).waitSeconds(2.75)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(3.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .endTrajectory();


        TrajectoryActionBuilder redTraj = driveBase.actionBuilder(mapR.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(-64.0, -33.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr);
                    spinSimple.track(Hr2, 0.0);
                    spinSimple.hoodAngle(HAr);

                }).waitSeconds(2.75)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(-11.45, -22.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-11.45, -47.0))
                .waitSeconds(1.0)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    spinSimple.track(0.0, 0.0);

                })
                .waitSeconds(1.0)
//                .splineToConstantHeading(new Vector2d(-64.0, -33.5), Math.toRadians(90.0))
                .strafeToConstantHeading(new Vector2d(-64.0, -33.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr);
                    spinSimple.track(Hr2, 0.0);
                    spinSimple.hoodAngle(HAr);

                }).waitSeconds(2.75)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(3.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .endTrajectory();


        TrajectoryActionBuilder blueTraj2 = driveBase.actionBuilder(mapB.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(60.0, 17.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSb2);
                    spinSimple.track(Hb22, 3.0);
                    spinSimple.hoodAngle(HAb2);

                }).waitSeconds(4)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(35.95, 24.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.95, 52.0))
                .waitSeconds(1.0)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    spinSimple.track(0.0, 0.0);

                })
                .waitSeconds(1.0)
                .strafeToConstantHeading(new Vector2d(60.0, 17.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSb2);
                    spinSimple.track(Hb22, 3.0);
                    spinSimple.hoodAngle(HAb2);

                }).waitSeconds(4)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                })
                .waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(35.95, 24.0))
                .endTrajectory();

//        TrajectoryActionBuilder blueTraj2Part2 = driveBase.actionBuilder(mapB.getParkPosition())
//                .waitSeconds(1.0)
//                .strafeToConstantHeading(new Vector2d(60.0, 17.5))
//                .stopAndAdd(() -> {
//
//                    outtake.setPower(VSb2);
//                    spinSimple.track(Hb22, 3.0);
//                    spinSimple.hoodAngle(HAb2);
//
//                }).waitSeconds(4)
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
//                    spinSimple.track(0.0, 0.0);
//                    spinSimple.hoodAngle(10.0);
//                    outtake.close();
//
//                })
//                .waitSeconds(0.75)
//                .strafeToConstantHeading(new Vector2d(35.95, 24.0))
//                .endTrajectory();

        TrajectoryActionBuilder redTraj2 = driveBase.actionBuilder(mapR.getStartPosition()) // put initial pose here
                .strafeToConstantHeading(new Vector2d(60.0, -17.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr2);
                    spinSimple.track(Hr22, -3.0);
                    spinSimple.hoodAngle(HAr2);

                }).waitSeconds(4)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                }).waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(35.95, -24.0))
                .stopAndAdd(() -> {

                    intake.intake();

                })
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(35.95, -52.0))
                .waitSeconds(1.0)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    spinSimple.track(0.0, 0.0);

                })
                .waitSeconds(1.0)
                .strafeToConstantHeading(new Vector2d(60.0, -17.5))
                .stopAndAdd(() -> {

                    outtake.setPower(VSr2);
                    spinSimple.track(Hr22, -3.0);
                    spinSimple.hoodAngle(HAr2);

                }).waitSeconds(4)
                .stopAndAdd(() -> {

                    intake.intake();
                    outtake.open();

                }).waitSeconds(2.5)
                .stopAndAdd(() -> {

                    intake.stopIntake();
                    outtake.stopShoot();
                    spinSimple.track(0.0, 0.0);
                    spinSimple.hoodAngle(10.0);
                    outtake.close();

                }).waitSeconds(0.75)
                .strafeToConstantHeading(new Vector2d(35.95, -24.0))
                .endTrajectory();

//        TrajectoryActionBuilder redTraj2Part2 = driveBase.actionBuilder(mapR.getParkPosition())
//                .waitSeconds(1.0)
//                .strafeToConstantHeading(new Vector2d(60.0, -17.5))
//                .stopAndAdd(() -> {
//
//                    outtake.setPower(VSr2);
//                    spinSimple.track(Hr22, -3.0);
//                    spinSimple.hoodAngle(HAr2);
//
//                }).waitSeconds(4)
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
//                    spinSimple.track(0.0, 0.0);
//                    spinSimple.hoodAngle(10.0);
//                    outtake.close();
//
//                }).waitSeconds(0.75)
//                .strafeToConstantHeading(new Vector2d(35.95, -24.0))
//                .endTrajectory();

        TrajectoryActionBuilder parkTrajBlue = driveBase.actionBuilder(mapB.getParkPosition())
                .waitSeconds(1.0)
                        .endTrajectory();

        TrajectoryActionBuilder parkTrajRed = driveBase.actionBuilder(mapR.getParkPosition())
                .waitSeconds(1.0)
                .endTrajectory();

        waitForStart();

        Action traj1 = (isRed ? (innerPosition ? redTraj2.build() : redTraj.build()) : (innerPosition ? blueTraj2.build() : blueTraj.build()));

//        Action traj2 = (isRed ? redTraj2Part2.build() : blueTraj2Part2.build());
//
//        Action park = (isRed ? parkTrajRed.build() : parkTrajBlue.build());

        Actions.runBlocking(
                new SequentialAction(
                        traj1
//                        (sixBall ? traj2 : park)
                )
        );


        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            PoseStorage.currentPose = new Pose2d(driveBase.localizer.getPose().position.x, - driveBase.localizer.getPose().position.y, driveBase.localizer.getPose().heading.toDouble());

//            PoseStorage.currentPose = driveBase.localizer.getPose();

            Logging.LOG("X coordinate (IN)", driveBase.localizer.getPose().position.x);
            Logging.LOG("Y coordinate (IN)", driveBase.localizer.getPose().position.y);
            Logging.LOG("Heading angle (DEGREES)", Math.toDegrees(driveBase.localizer.getPose().heading.toDouble()));

            Logging.update();
        }
    }
}