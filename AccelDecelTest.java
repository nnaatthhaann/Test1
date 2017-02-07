package org.firstinspires.ftc.teamcode;

import android.provider.Telephony;
import android.test.InstrumentationTestRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@Autonomous (name = "AccelDecel Test 1.4")
@Disabled
public class AccelDecelTest extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    private DcMotor shooter;

    @Override
    public void runOpMode() throws InterruptedException
    {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");

        //shooter = hardwareMap.dcMotor.get("shooter");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        //establishes default magic number for drive power
        double POWER = 1;

        waitForStart();

        //START AUTONOMOUS

        Drive(POWER, 1500);
        TurnTime(POWER, 500);

        idle();
    }

    private void Drive(double maxpower, long time)throws InterruptedException
    {
        double out;
        int t = 0;        //t = runtime

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
        for (double z = .1; z != 0.0; z -= .1)
        {
            out = z;

            front_left.setPower(out);
            back_left.setPower(out);
            front_right.setPower(out);
            back_right.setPower(out);
            Thread.sleep(50);
        }

        Thread.sleep(150);
    }

    private void DriveForward(double power)
    {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(power);
    }

    private void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    private void TurnTime(double power, long time) throws InterruptedException
    {
        //turning is slower for accuracy
        power *= .7;

        //default turn left
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);

        Drive(power, time);

        //resets to previous state
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
    }

    private void BackUpTime(double power, long time) throws InterruptedException
    {
        Drive(-power, time);
    }

    /*
    private void StopMotors() throws InterruptedException
    {
        DriveForward(0.0);
        Thread.sleep(100);
    }
    */
}
