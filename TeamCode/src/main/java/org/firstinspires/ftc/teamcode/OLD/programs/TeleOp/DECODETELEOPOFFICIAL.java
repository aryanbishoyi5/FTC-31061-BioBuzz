
package org.firstinspires.ftc.teamcode.programs.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanims.AprilTags;
import org.firstinspires.ftc.teamcode.mechanims.robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "DECODE TELEOP OFFICIAL", group = "TeleOp")
public class DECODETELEOPOFFICIAL extends OpMode {
    final double FEED_TIME_SECONDS = 0.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;
    robot robot;

    double sensitivityMultiplier = 1;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    AprilTags aprilTagWebcam = new AprilTags();

    String activelyShooting = "Not Spinning";

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;



    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    @Override
    public void init() {
        //zeroPowerShoot(true);
        robot = new robot(hardwareMap, telemetry);
        robot.odo.resetPosAndIMU();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "sorter");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        //leftFeeder.setPower(1.0);
        //rightFeeder.setPower(-1.0);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);
        aprilTagWebcam.displayDetectionTelemetry(id21);
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);
        aprilTagWebcam.displayDetectionTelemetry(id22);
        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);
        aprilTagWebcam.displayDetectionTelemetry(id23);

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(0.9);
                    break;
            }
        }
        if (gamepad1.left_bumper) {
            sortLeft();
        }
        if (gamepad1.right_bumper) {
            sortRight();
        }
        maxPowerShoot(gamepad2.y);
        minPowerShoot(gamepad2.x);
        zeroPowerShoot(gamepad2.b);
        leftLaunchHyp(gamepad2.left_trigger>0);
        rightLaunchHyp(gamepad2.right_trigger>0);
        sensitivityChanger(gamepad1.left_trigger>0);

        telemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
        telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
        telemetry.addData("Sorter Pos", diverterTelemetry());
        telemetry.addData("Is INSPIRAD shooting?", activelyShooting);
        telemetry.addData("Sensitivity Proportion", sensitivityMultiplier);
        telemetry.update();
    }

    String diverterTelemetry () {
        if (diverter.getPosition() == 0) {
            return "Left Position";
        } else if (diverter.getPosition() == 0.2962) {
            return "Right Position";
        }
        return "";
    }

    double shootPowerCalc(AprilTagDetection tags) {
        double x = tags.ftcPose.x;
        double y = tags.ftcPose.y;
        double distance = Math.sqrt(x * x + y * y);
        double multiplier = 3;
        return multiplier*distance;

    }

    void mecanumDrive(double forward, double strafe,      double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = ((forward - strafe + rotate)*sensitivityMultiplier) / denominator;
        rightFrontPower = ((forward + strafe - rotate)*sensitivityMultiplier) / denominator;
        leftBackPower = ((forward + strafe + rotate)*sensitivityMultiplier) / denominator;
        rightBackPower = ((forward - strafe - rotate)*sensitivityMultiplier) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        telemetry.addData("strafe", strafe);

    }

    void sortLeft () {
        diverter.setPosition(RIGHT_POSITION);
    }
    void sortRight () {
        diverter.setPosition(LEFT_POSITION);
    }
    void leftLaunchHyp(Boolean shotRequested) {
        if (shotRequested) {
            leftFeeder.setPower(1);
        } else {
            leftFeeder.setPower(-1);
        }
    }
    void maxPowerShoot (Boolean shotRequested) {
        double maxPower = 1550;
        if (shotRequested) {
            leftLauncher.setVelocity(maxPower+100);
            rightLauncher.setVelocity(maxPower);
            activelyShooting = "Yes," + maxPower + "RPM max";
        }
        activelyShooting = "No, 0 RPM max";
    }
    void minPowerShoot (Boolean shotRequested) {
        double minPower = 1370;
        if (shotRequested) {
            leftLauncher.setVelocity(minPower);
            rightLauncher.setVelocity(minPower);
            activelyShooting = "Yes," + minPower + "RPM max";
        }
        activelyShooting = "No, 0 RPM max";
    }
    void zeroPowerShoot (Boolean shotRequested) {
        if (shotRequested) {
            leftLauncher.setVelocity(STOP_SPEED);
            rightLauncher.setVelocity(STOP_SPEED);
        }
    }
    void rightLaunchHyp(Boolean shotRequested) {
        if (shotRequested) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(-1);
        }
    }
    void sensitivityChanger (Boolean changeRequested) {
        if (changeRequested) {
            sensitivityMultiplier = 0.3;
        } else {
            sensitivityMultiplier = 1;
        }
    }
}