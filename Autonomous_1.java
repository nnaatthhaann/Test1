package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@Autonomous (name = "Auton 1.38 RED")
@Disabled
public class Autonomous_1 extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    private DcMotor shooter;

    //beacon pushing servos
    private Servo beaconleft = null;
    private Servo beaconright = null;

    //sensors
    private OpticalDistanceSensor beaconods;
    private OpticalDistanceSensor lineods;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");

        shooter = hardwareMap.dcMotor.get("shooter");

        //servos
        beaconleft = hardwareMap.servo.get("left");
        beaconright = hardwareMap.servo.get("right");

        //sensors
        beaconods = hardwareMap.opticalDistanceSensor.get("beaconods");
        lineods = hardwareMap.opticalDistanceSensor.get("lineods");
        colorSensor = hardwareMap.colorSensor.get("color");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        double beaconRaw;
        double beaconDistance = 35;

        double lineRaw = lineods.getRawLightDetected();
        double lineLinear;

        waitForStart();

        //START AUTONOMOUS

        Drive(POWER * .5, 1000);                 //drive away from wall
        TurnRightTime(TURN_POWER, 1220);        //turn 45 deg to the right
        Drive(POWER, 4000);                     //drive to beacon line thingy

        // track and push beacon
        while (lineRaw < 700)
        {
            DriveForward(.15);
        }
        DriveForward(0.0);

        while (beaconDistance < 1.5)
        {
            beaconRaw = beaconods.getRawLightDetected();                            //reads sensor value
            beaconDistance = 31.968723726291 * (Math.pow(beaconRaw, -.5));          //makes progression linear

            lineRaw = lineods.getRawLightDetected() / 5;                            //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            lineLinear = Math.pow(lineRaw, 0.5);

            front_left.setPower(lineLinear * 2);
            back_left.setPower(lineLinear * 2);
            front_right.setPower(0.5 - (lineLinear * 2));
            back_right.setPower(0.5 - (lineLinear * 2));
        }

        //color sensor must be on left side
        if (colorSensor.red() <= colorSensor.blue()) {
            beaconright.setPosition(1);     //push button
        }
        else {
            beaconleft.setPosition(0);      //push button
        }

        Drive(-POWER, 1300);                    //back away from target beacon
        TurnRightTime(TURN_POWER, 2500);        //turn 90 deg
        ShootTime(3000);                        //shoot 2 balls
        Drive(-POWER * 2,3000);                         //back up
        TurnRightTime(TURN_POWER, 1220);        //turn 45 deg to the right
        Drive(-1, 2000);                       //hit cap ball
        StopMotors();                           //end autonomous

        idle();
    }

    //establishes default magic number for drive power
    double POWER = .5;
    double TURN_POWER = .2;

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

    private void StopMotors() throws InterruptedException
    {
        DriveForward(0.0);
    }

    private void pause() throws InterruptedException
    {
        StopMotors();
        Thread.sleep(500);
    }

     private void Drive(double maxpower, long time)throws InterruptedException
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
        wait(time);
    }
}
