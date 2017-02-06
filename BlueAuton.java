package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@Autonomous (name = "BlueAuton")
public class BlueAuton extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor collector;
    private DcMotor shooter;

    //servos
    private Servo beacon;

    //sensors
    private ModernRoboticsAnalogOpticalDistanceSensor floorODS;
    private ModernRoboticsI2cColorSensor color;
    private ModernRoboticsAnalogOpticalDistanceSensor wallODS;
    private ModernRoboticsI2cRangeSensor range;

    @Override
    public void runOpMode() throws InterruptedException
    {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        beacon = hardwareMap.servo.get("beacon");

        floorODS = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods1");
        wallODS = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");
        color = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        color.enableLed(false);     //Set color sensor to passive mode

        waitForStart();

        //START AUTONOMOUS

        //initiate at starting point
        Drive(POWER, 500);
        TurnRightTime(TURN_POWER, 1600);
        DriveForward(1);
        TurnRightTime(-TURN_POWER, 1600);

        //first beacon
        DriveForward(.4);
        while (floorODS.getRawLightDetected() < 800)
            Thread.sleep(50);
        pause();
        Drive(POWER, 400);

        if (color.blue() > color.red() && color.blue() > color.green()) {
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != Servo.MIN_POSITION) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        }
        else {
            Drive(POWER, 500);
            wait(100);
            beacon.setPosition(Servo.MIN_POSITION);
            while (beacon.getPosition() != 0) {
            }
            beacon.setPosition(Servo.MAX_POSITION);
        }

        /*
        Drive(1, 3500);

        //second beacon
        DriveForward(.4);
        while (floorODS < 800)
            Thread.sleep(50);
        pause();
        Drive(POWER, 400);

        if (color.blue() > color.red()) {
            beacon.setPosition(MIN_POS);
            while (beacon.getPosition != 0) {
            }
            beacon.setPosition(MAX_POS);
        }
        else {
            Drive(POWER, 500)
            beacon.setPosition(MIN_POS);
            while (beacon.getPosition != 0) {
            }
            beacon.setPosition(MAX_POS);
        }
        waitOneFullHardwareCycle();
        */

        idle();
    }

    //establishes default magic number for drive power
    double TURN_POWER = .2;
    double POWER = .5;

    private void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    private void DriveForward(double power)
    {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(power);
    }

    private void TurnRightTime(double power, long time) throws InterruptedException
    {
        //default right
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(-power);

        Thread.sleep(time);

        pause();
    }
    /*
        private void PauseShooter (double power, long time) throws InterruptedException
        {
            shooter.setPower(0);
            Thread.sleep(time);
        }
        */
    private void TurnLeftTime(double power, long time) throws InterruptedException
    {
        //default right
        front_left.setPower(-power);
        back_left.setPower(-power);
        front_right.setPower(power);
        back_right.setPower(power);

        Thread.sleep(time);

        pause();
    }

    private void StopMotors() throws InterruptedException
    {
        DriveForward(0.0);
    }

    private void pause() throws InterruptedException
    {
        StopMotors();
        Thread.sleep(500);
    }

    private void Drive(double maxpower, long time) throws InterruptedException
    {
        double out;
        int t = 0;        //t = runtime

        //Decceleration of motors
        if (maxpower > 0) //if power is pos
        {
            //Acceleration of the 4 drive motors
            for (double x = .1; x <= maxpower; x += .1)
            {
                out = x;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);

           /*
           Every 50 milis add one value to variable 'time'
           Thus, resulting 'time' is length of runtime in sets of 50 milis
           */
                t = t + 1;
            }

            DriveForwardTime(maxpower, time - 2 * (50 * t));

            //Decceleration of motors
            for (double z = maxpower; z >= .05; z -= .05) {
                out = z;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);
            }
        }
        else        //if power is neg
        {
            //Acceleration of the 4 drive motors
            for (double x = -.1; x >= maxpower; x -= .1)
            {
                out = x;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);

           /*
           Every 50 milis add one value to variable 'time'
           Thus, resulting 'time' is length of runtime in sets of 50 milis
           */
                t = t + 1;
            }

            DriveForwardTime(maxpower, time - 2 * (50 * t));

            //Decceleration of motors
            for (double z = maxpower; z <= -.05; z += .05)
            {
                out = z;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);
            }
        }

        pause();
    }

    private void ShootTime(long time)throws InterruptedException
    {
        shooter.setPower(1);
        Thread.sleep(time);
    }
}
