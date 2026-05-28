package org.firstinspires.ftc.teamcode.mechanims;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class robot {
    public mecanumDrive mecanum;
    public GoBildaPinpointDriver odo;
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    AprilTags aprilTagWebcam = new AprilTags();

    public static Pose2d startPos = new Pose2d(0, 0, 0);
    List<LynxModule> allHubs;

    //green is left compartment
    //purple is right compartment
    boolean GPP = true;
    boolean PGP = false;
    boolean PPG = false;

    public DcMotor intake;



    public robot(HardwareMap map, Telemetry telemetry) {
        this.hardwareMap = map;
        this.telemetry = telemetry;
        startPos = new Pose2d(0, 0, 0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        mecanum = new mecanumDrive(hardwareMap, telemetry);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        telemetry.update();
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void stopIntake () {
        intake.setPower(0);
    }

    public void startShoot (double power) {
        DcMotorEx leftLauncher;
        DcMotorEx rightLauncher;
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(320, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(320, 0, 0, 10));

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setVelocity(power);
        rightLauncher.setVelocity(power);
    }

    public void detectOrder () {
        aprilTagWebcam.buildVisionPortal(hardwareMap, telemetry);
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);
        aprilTagWebcam.displayDetectionTelemetry(id21);
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);
        aprilTagWebcam.displayDetectionTelemetry(id22);
        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);
        aprilTagWebcam.displayDetectionTelemetry(id23);
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);

        if (id21 != null) {
            GPP = true;
            PGP = false;
            PPG = false;
        } else if (id22 != null) {
            GPP = false;
            PGP = true;
            PPG = false;
        } else if (id23 != null) {
            GPP = false;
            PGP = false;
            PPG = true;
        }
    }

    public void shoot() {
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        ElapsedTime leftFeederTimer = new ElapsedTime();
        ElapsedTime rightFeederTimer = new ElapsedTime();
        double FULL_SPEED =-1.0;
        double STOP_SPEED =0.0;
        double FEED_TIME_SECONDS = 2;
        while (leftFeederTimer.seconds() < FEED_TIME_SECONDS) {
            leftFeeder.setPower(-FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);
            mecanum.brake();
        }
    }

    public void inverseTransfer() {
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
    }

    public void stopTransfer() {
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    public void shootInOrder () {
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        ElapsedTime leftFeederTimer = new ElapsedTime();
        ElapsedTime rightFeederTimer = new ElapsedTime();
        double FULL_SPEED =-1.0;
        double STOP_SPEED =0.0;
        double FEED_TIME_SECONDS = .5;
        if (GPP) {
            leftFeederTimer.reset();
            while (leftFeederTimer.seconds() < FEED_TIME_SECONDS) {
                leftFeeder.setPower(-FULL_SPEED);
            }
            leftFeeder.setPower(STOP_SPEED);
            rightFeederTimer.reset();
            while (rightFeederTimer.seconds() < (FEED_TIME_SECONDS*2)) {
                leftFeeder.setPower(-FULL_SPEED);
            }
            rightFeeder.setPower(STOP_SPEED);
        } else if (PGP) {
            rightFeederTimer.reset();
            while (rightFeederTimer.seconds() < FEED_TIME_SECONDS) {
                rightFeeder.setPower(FULL_SPEED);
            }
            rightFeeder.setPower(STOP_SPEED);
            leftFeederTimer.reset();
            while (leftFeederTimer.seconds() < FEED_TIME_SECONDS) {
                leftFeeder.setPower(-FULL_SPEED);
            }
            leftFeeder.setPower(STOP_SPEED);
            rightFeederTimer.reset();
            while (rightFeederTimer.seconds() < FEED_TIME_SECONDS) {
                rightFeeder.setPower(FULL_SPEED);
            }
            rightFeeder.setPower(STOP_SPEED);
        } else if (PPG) {
            rightFeederTimer.reset();
            while (rightFeederTimer.seconds() < (FEED_TIME_SECONDS*2)) {
                rightFeeder.setPower(FULL_SPEED);
            }
            rightFeeder.setPower(STOP_SPEED);
            leftFeederTimer.reset();
            while (leftFeederTimer.seconds() < FEED_TIME_SECONDS) {
                leftFeeder.setPower(-FULL_SPEED);
            }
            leftFeeder.setPower(STOP_SPEED);
        }
    }

    public void GoTo(Pose2d pose, Pose2d speedLimits) {
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z) {
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    public boolean atTarget(Pose2d target) {
        double diff_x = target.getX() + (odo.getPosX(DistanceUnit.MM) / DistanceUnit.mmPerInch);
        double diff_y = target.getY() - (odo.getPosY(DistanceUnit.MM) / DistanceUnit.mmPerInch);
        double diff_h = target.getHeading() - odo.getHeading(AngleUnit.RADIANS);

        diff_h = Math.atan2(Math.sin(diff_h), Math.cos(diff_h));

        return Math.abs(diff_x) < 1 && Math.abs(diff_y) < 1 && Math.abs(diff_h) < 0.1;
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits) {
        mecanum.goToPoint(pose, new Pose2d(-odo.getPosX(DistanceUnit.MM) / DistanceUnit.mmPerInch, odo.getPosY(DistanceUnit.MM) / DistanceUnit.mmPerInch, odo.getHeading(AngleUnit.RADIANS)), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", new Pose2d(odo.getPosX(), odo.getPosY(), odo.getHeading()));
        telemetry.addData("Target Position: ", pose);
        mecanum.write();
        odo.update();
    }
}