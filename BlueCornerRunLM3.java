package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@Autonomous (name = "Blue Corner Run")
public class BlueCornerRunLM3 extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor collector;
    private DcMotor shooter;


    @Override
    public void runOpMode() throws InterruptedException
    {  front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        //START AUTONOMOUS
        //initiate at starting point A
        Thread.sleep(10000);
        DriveForwardTime(POWER, 1800); //drive forward
        StopMotors();//stop
        TurnLeftTime(POWER, 1200); //move forward
        ShootTime(1000); //shoot
        ShootTime(0);
        Thread.sleep(2000);
        //StopMotors();//stop
        collector.setPower(1);//run collector
        Thread.sleep(3000);//
        ShootTime(1000);//shoot
        //idle();

        //Thread.sleep(4000);

        TurnRightTime(POWER, 1300);
        DriveForwardTime(POWER, 1400);

    }

    //establishes default magic number for drive power

    double TURN_POWER = .2;double POWER = .5;

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
            Thread.sleep(time);
    }
}

