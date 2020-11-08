package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

@TeleOp(name = "10/20/20")
public class TeleOpCool extends OpMode {
    private Motor fL, bL, fR, bR; //eL, eR
    private MecanumDrive drive;
    private RevIMU imu;
    GamepadEx gPad;
    //DifferentialOdometry odom;

    @Override
    public void init() {
        //motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        //encoders
        /*
        eL = new Motor(hardwareMap, "eL");
        eR = new Motor(hardwareMap, "eR");
        */

        drive = new MecanumDrive(fL, fR, bL, bR);
        imu = new RevIMU(hardwareMap);

        gPad = new GamepadEx(gamepad1);
    }
    @Override
    public void loop() {

        drive.driveRobotCentric(gPad.getLeftX(), gPad.getLeftY(), gPad.getRightX());

        telemetry.addData("Strafe Speed:", gPad.getLeftX());
        telemetry.addData("Forward Speed:", gPad.getLeftY());
        telemetry.addData("Turn Speed:", gPad.getRightX());
        telemetry.update();
    }
}