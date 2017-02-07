/*
Modern Robotics ODS Encoder Example1
Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor named "ods1"

This program can be run without a battery and Power Destitution Module.

View the video about this at https://youtu.be/EuDYJPGOOPI.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Sensor: ODS", group = "sensorTest")
@Disabled
public class ODSTest extends LinearOpMode {

    //sensor value between 0 and 1023
    int raw1;
    private OpticalDistanceSensor ods1;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    @Override
    public void runOpMode() {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");

        waitForStart();

        while (opModeIsActive()) {
            raw1 = (int) (ods1.getLightDetected() * 1023);

            idle();
        }
    }

    public void FollowLine() {
        raw1 = (int) (ods1.getLightDetected() * 1023);

        double power = front_left.getPower();
        //sets proportional 'servo control' for smooth line following
        rightDrive(power - (800 - raw1) / 1000);
        leftDrive(power + (800 - raw1) / 1000);
    }

    public void leftDrive(double power) {
        front_left.setPower(power);
        back_left.setPower(power);
    }

    public void rightDrive(double power) {
        front_right.setPower(power);
        back_right.setPower(power);
    }
}
