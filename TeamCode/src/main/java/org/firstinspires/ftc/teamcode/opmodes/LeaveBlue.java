package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideLeaveBlue;
import com.userjhansen.automap.Maps.InsideLeaveRed;
import com.userjhansen.automap.Maps.InsideOneRed;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideLeaveBlue;
import com.userjhansen.automap.Maps.OutsideLeaveRed;
import com.userjhansen.automap.Maps.OutsideOneRed;

import org.firstinspires.ftc.teamcode.MecanumDrive;
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

@Autonomous(name = "LeaveBlue")
@Config
public class LeaveBlue extends LinearOpMode {
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
                    traj = traj.turn(part.value + 0.00001);
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
        Limelight ll = new Limelight(hardwareMap, "limelight");
        TurretSimple spinSimple = new TurretSimple(hardwareMap);


//        LLResult result = visionDetection.limelight.getLatestResult();
//        LLResultTypes.FiducialResult fr = (LLResultTypes.FiducialResult) result.getFiducialResults();
//
//        int tagId = fr.getFiducialId();
//
//        Motif motif = Motif.None;
//
//        if (result.isValid()){
//            fr.getFiducialId();
//            if (fr.getFiducialId() == 21){
//                motif = Motif.GPP;
//            } else if (fr.getFiducialId() == 22){
//                motif = Motif.PGP;
//            } else if (fr.getFiducialId() == 23){
//                motif = Motif.PPG;
//            }
//            switch (motif) {
//                case GPP:
//                    // Go to corresponding group of artifacts on field
//                    break;
//                case PGP:
//                    // Go to corresponding group of artifacts on field
//                    break;
//                case PPG:
//                    // Go to corresponding group of artifacts on field
//                    break;
//            }
//        } else {
//            Logging.LOG("No Target Found");
//        }

//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//
//        Pose2D pose = pinpoint.getPosition();
//
////      set to ending auto pose
//        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//
        Pose2d pose = driveBase.localizer.getPose();

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Action initAction = new ParallelAction(

        );

//        Run initialisation tasks
        visionDetection.limelight.start();
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }


        boolean innerPosition = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
//            pinpoint.update();

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.y) innerPosition = false;

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;

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

        double X = driveBase.localizer.getPose().position.x;

        double Y = driveBase.localizer.getPose().position.y;

        double H = driveBase.localizer.getPose().heading.toDouble();

        double Dr = Math.sqrt(((72.0 - Y) * (72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

        double Db = Math.sqrt(((-72.0 - Y) * (-72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

        double Hr3 = Math.atan((72.0 + X) / (72.0 - Y)) - H;

        double Hb3 = Math.atan((72.0 + X) / (-72.0 - Y)) + Math.toRadians(180.0) - H;

        double Hr2 = Math.atan2((72.0 + X), (72.0 - Y)) - H;

        double Hb2 = Math.atan2((72.0 + X), (-72.0 - Y)) + Math.toRadians(180.0) - H;

        double Hr = Math.acos((72.0 - Y) / Dr) - H;

        double Hb = Math.acos((-72.0 - Y) / Db) - H;

        double Hm = 90.0 - H;

        double RSr = (-0.032) * Dr * Dr + 11.68 * Dr + 386.0;

        double RSb = (-0.032) * Db * Db + 11.68 * Db + 386.0;

        Map map = innerPosition ? new InsideLeaveBlue() : new OutsideLeaveBlue();

        driveBase.localizer.setPose(!PoseStorage.isRedAlliance
                ? map.getStartPosition()
                : new Pose2d(
                map.getStartPosition().position.x,
                -map.getStartPosition().position.y,
                map.getStartPosition().heading.plus(Math.PI).toDouble()));

//        driveBase.localizer.setPose(map.getStartPosition());

        // make sure to stop all actions

        TrajectoryActionBuilder wholeTrajectory = driveBase.allianceActionBuilder(map.getStartPosition()) // put initial pose here
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(34.2, -19))
                .endTrajectory();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        wholeTrajectory.build()
                )
        );
    }
}