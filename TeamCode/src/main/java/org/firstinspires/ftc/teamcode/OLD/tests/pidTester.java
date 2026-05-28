package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanims.robot;

@Autonomous
public class pidTester extends OpMode {
    robot robot;

    @Override
    public void init() {
        robot = new robot(hardwareMap, telemetry);
        robot.odo.resetPosAndIMU();
    }

    @Override
    public void loop() {
        robot.GoTo(new Pose2d(PID_Tuner_Constants.x, PID_Tuner_Constants.y, Math.toRadians(PID_Tuner_Constants.theta)), new Pose2d(1, 1, 1));

    }
}

@Config
class PID_Tuner_Constants {
    public static double x = 0;
    public static double y = 0;
    public static double theta = 0;
}