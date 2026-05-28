package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanims.mecanumDrive;

@TeleOp
public class driveTester extends OpMode {
    mecanumDrive drive;

    @Override
    public void init() {
        drive = new mecanumDrive(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1,1,1);
        drive.write();
    }
}
