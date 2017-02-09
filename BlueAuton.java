package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@Autonomous (name = "BlueAuton")
public class BlueAuton extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    private DcMotor collector;

    private DcMotor shooter;

    private Servo beacon;

    private ModernRoboticsAnalogOpticalDistanceSensor floorODS;
    //private ModernRoboticsAnalogOpticalDistanceSensor wallODS;        //replaced by range
    private ModernRoboticsI2cColorSensor color;
    private ModernRoboticsI2cRangeSensor range;
    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");

        collector = hardwareMap.dcMotor.get("collector");

        shooter = hardwareMap.dcMotor.get("shooter");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        color.enableLed(false);         //set color sensor to passive mode

        //int zAccumulated;
        //int target = 0;
        //double TURN_POWER = .2;
        double POWER = .5;
        //double distance;

        //gyro sensor calib
        telemetry.addData("Gyro Sensor:", "Calibrating... Keep robot still");
        gyro.calibrate();

        while (gyro.isCalibrating()) {
        }
        telemetry.addData("Gyro Sensor:", "Done");
        telemetry.addData("Status:", "Initialized");

        waitForStart();

        double distance;

        DriveForwardTime(.5, 500);
        turnDegrees(30);
        DriveForward(1);
        while (range.getDistance(DistanceUnit.CM) > 5) {
        }
        pause();

        //align with wall
        while (gyro.getIntegratedZValue() > 0) {
            right(0.5);
            left(0.1);
        }

        //approach first beacon
        //drive to line
        while (floorODS.getRawLightDetected() < 800) {
            distance = range.getDistance(DistanceUnit.CM);

            //will maintain 8 cm from wall
            left(-distance / 16);
            right(-(1.0 - (distance / 16)));
        }
        pause();

        //align with target
        DriveForwardTime(POWER, 400);

        //test for beacon color & press button
        if (color.blue() > color.red()) {
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != Servo.MIN_POSITION) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        } else {
            DriveForwardTime(POWER, 500);
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != 0) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        }

        //follow wall to next line
        while (floorODS.getRawLightDetected() < 800) {
            distance = range.getDistance(DistanceUnit.CM);

            //will maintain 8 cm from wall
            left(distance / 16);
            right(1.0 - (distance / 16));
        }

        //align with beacon
        DriveForwardTime(POWER, 400);

        //test for beacon color & press button
        if (color.blue() > color.red()) {
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != Servo.MIN_POSITION) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        } else {
            DriveForwardTime(-POWER, 500);
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != 0) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        }

        //move away from wall
        //face hoop
        strafeLeft(2000);
        turnDegrees(45);

        //approach hoop
        //shoot 2 balls
        DriveForwardTime(-1, 1500);
        turnDegrees(90);
        shootNumber(2);

        //turn to cap ball
        turnDegrees(90);
        DriveForwardTime(1, 500);

        /*

        BAM ez
        (2) Beacon              60pts
        (2) Center Vortex       30pts
            CapBall             5pts
            Center Parking      5pts
            ------------------- 100pts

        */
    }

    private void DriveForwardTime(double power, long time) {
        runtime.reset();
        runtime.startTime();
        do {
            DriveForward(power);
        } while (runtime.time() < time);
        DriveForward(0);
    }

    private void DriveForward(double power) {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(power);
    }

    private void pause() {
        DriveForward(0);

        runtime.reset();
        runtime.startTime();
        while (runtime.time() < 250) {
        }
    }

    private void turnDegrees(int target) {
        turnAbsolute(target + gyro.getIntegratedZValue());
    }

    private void turnAbsolute(int target) {
        int zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings

        double turnSpeed = 0.3;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn left
                front_left.setPower(-turnSpeed);
                back_left.setPower(-turnSpeed);
                front_right.setPower(turnSpeed);
                front_right.setPower(turnSpeed);
            }

            if (zAccumulated < target) {
                front_left.setPower(turnSpeed);
                back_left.setPower(turnSpeed);
                front_right.setPower(-turnSpeed);
                front_right.setPower(-turnSpeed);
            }
        }
        pause();
    }

    private void shootNumber(int balls) {
        int num;
        for (num = 0; num <= balls; num++) {
            shootTime(1000);

            do {
                collector.setPower(1);
            } while (runtime.time() < 3000);
            collector.setPower(0);

            pause();
        }
    }

    private void shootTime(long time) {
        runtime.reset();
        runtime.startTime();
        while (runtime.time() < time) {
            shooter.setPower(1);
        }
        shooter.setPower(0);
    }

    private void strafeLeft(long time) {
        runtime.reset();
        runtime.startTime();
        while (runtime.time() < time) {
            double heading = gyro.getHeading();

            front_left.setPower(heading / 100);
            front_right.setPower(-heading / 100);
            back_left.setPower(-(1.0 - (heading / 100)));
            back_right.setPower(1.0 - (heading / 100));
        }
        pause();
    }

    private void left(double power) {
        front_left.setPower(power);
        back_left.setPower(power);
    }

    private void right(double power) {
        back_right.setPower(power);
        front_right.setPower(power);
    }
}
