package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ramroids 10318 on 1/24/2017.
 */

@Autonomous (name = "Gyro_Test 1.1")
@Disabled
class GyroTestDrive_1 extends LinearOpMode {

    private DcMotor frontleft; //front left
    private DcMotor frontright; //front right
    private DcMotor backleft; //back left
    private DcMotor backright; //back right

    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.dcMotor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating... Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        int heading = gyro.getHeading();


        idle();
    }

    private void frontMotors(double power) {
        frontleft.setPower(power);
        frontright.setPower(power);
    }

    private void backMotors(double power) {
        backleft.setPower(power);
        backright.setPower(power);
    }

    /*
    private void gyroStrafe(double power, long time) throws InterruptedException{

        while () {

        }
            frontMotors(power - );
            backMotors();
    }
    */
}
