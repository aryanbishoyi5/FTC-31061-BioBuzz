package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanims.AprilTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "APRIL TAG TESTER", group = "TeleOp")
public class aprilTagsTester extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    AprilTagDetection id21;
    AprilTagDetection id22;
    AprilTagDetection id23;
    AprilTagDetection id24;
    AprilTagDetection id25;


    AprilTags aprilTags = new AprilTags();
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        aprilTags.init(hardwareMap, telemetry);
    }

    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        aprilTags.update();
        id21 = aprilTags.getTagBySpecificId(21);
        id22 = aprilTags.getTagBySpecificId(22);
        id23 = aprilTags.getTagBySpecificId(23);

        AprilTagDetection id20 = aprilTags.getTagBySpecificId(20);
        AprilTagDetection id24 = aprilTags.getTagBySpecificId(24);

        aprilTags.displayDetectionTelemetry(id21);
        aprilTags.displayDetectionTelemetry(id22);
        aprilTags.displayDetectionTelemetry(id23);

        aprilTags.displayDetectionTelemetry(id20);
        aprilTags.displayDetectionTelemetry(id24);

        telemetry.addData("id20 String", id20.toString());
        telemetry.addData("id21 String", id21.toString());
        telemetry.addData("id23 String", id22.toString());
        telemetry.addData("id24 String", id23.toString());
        telemetry.addData("id25 String", id24.toString());


        if (id21 != null) {
            telemetry.addLine("Tag 21 detected -> GPP");
        } else if (id22 != null) {
            telemetry.addLine("Tag 22 detected -> PGP");
        } else if (id23 != null) {
            telemetry.addLine("Tag 23 detected -> PPG");
        } else {
            telemetry.addLine("No tag detected");
        }
        telemetry.update();

    }
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}