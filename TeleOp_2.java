package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ramroids 10318 on 12/26/2016.
 * ADDITIONS TO THIS VERSION
 * Added toggle system for the 2 separate drive modes.
 * When drivemode is "true" the drive is normal.
 * When it is "false" the power is scaled to a factor of .3 and left and right are reversed.
 * This is to provide an easier way for the driver to maneuver the robot w/ the cap ball.
 */

@TeleOp(name = "TeleOp 3.3")
@Disabled
public class TeleOp_2 extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    private DcMotor collector;

    private DcMotor shooter;

    private DcMotor lifter;
    private DcMotor lifter2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //INITIALIZE MOTORS AND THINGS

        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");

        collector = hardwareMap.dcMotor.get("collector");

        shooter = hardwareMap.dcMotor.get("shooter");

        lifter = hardwareMap.dcMotor.get("lifter");
        lifter2 = hardwareMap.dcMotor.get("lifter2");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        //establish variable for colletor toggle and drive mode
        boolean toggle = false;
        boolean drivemode = true;

        telemetry.addData("Drive Mode:", "NORMAL");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //RUN CODE

            //drive mode toggle
            boolean mCurrState = gamepad1.right_stick_button;
            boolean mPrevState = false;

            if ((mCurrState == true) && (mCurrState != mPrevState))
            {
                drivemode = !drivemode;
            }
            mCurrState = mPrevState;
            telemetry.addData("Drive Mode:", drivemode ? "NORMAL" : "CAP BALL");
            telemetry.update();

            if (drivemode)
            {
                while (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0
                        || gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0) {
                    front_left.setPower(-gamepad1.left_stick_y);
                    back_left.setPower(-gamepad1.left_stick_y);
                    front_right.setPower(-gamepad1.right_stick_y);
                    back_right.setPower(-gamepad1.right_stick_y);
                }
            }

            if (!drivemode)
            {
                while (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0
                        || gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0) {
                    back_right.setPower(gamepad1.left_stick_y * 0.3);
                    front_right.setPower(gamepad1.left_stick_y * 0.3);
                    back_left.setPower(gamepad1.right_stick_y * 0.3);
                    front_left.setPower(gamepad1.right_stick_y * 0.3);
                }
            }

            /*
            //left_trigger and right_trigger strafing
            if (gamepad1.left_trigger != 0.0 || gamepad1.right_trigger != 0.0) {
                //if left_trigger is activated, reverse ALL motors
                while (gamepad1.left_trigger < 0.0) {
                    front_left.setPower(gamepad1.left_trigger);
                    back_left.setPower(-gamepad1.left_trigger);
                    front_right.setPower(-gamepad1.left_trigger);
                    back_right.setPower(gamepad1.left_trigger);
                }

                //otherwise check for right_trigger
                front_left.setPower(-gamepad1.right_trigger);
                back_left.setPower(gamepad1.right_trigger);
                front_right.setPower(gamepad1.right_trigger);
                back_right.setPower(-gamepad1.right_trigger);
            }
            */

            //to fix motor 'creeping'
            if (gamepad1.left_stick_y == 0.0 || gamepad1.right_stick_y == 0.0) {
                front_left.setPower(0.0);
                back_left.setPower(0.0);
                front_right.setPower(0.0);
                back_right.setPower(0.0);
            }

            //collector toggle
            if (gamepad1.a) {
                //toggle state of boolean value
                toggle = !toggle;

                //turns on collector if value is true
                if (toggle) {
                    collector.setPower(1.0);
                }

                //waits for button to be reset
                Thread.sleep(250);
            }
            if (toggle == false) {
                collector.setPower(0.0);
            }

            //spits out ball
            while (gamepad1.b) {
                collector.setPower(-1.0);
            }

            //to fix motor 'creeping'
            if (gamepad1.left_stick_y == 0.0 || gamepad1.right_stick_y == 0.0) {
                front_left.setPower(0.0);
                back_left.setPower(0.0);
                front_right.setPower(0.0);
                back_right.setPower(0.0);
            }

            /*
            if (gamepad1.x )
            {
                //use encoder for shooter??
            }
            */

            while (gamepad1.x) {
                shooter.setPower(1.0);
            }
            while (gamepad1.y) {
                shooter.setPower(-1.0);
            }
            shooter.setPower(0.0);

            //Cap Ball Lifter
            if (gamepad1.dpad_up || gamepad1.dpad_down)
            {
                while (gamepad1.dpad_up) {
                    lifterPower(-1);
                }
                while (gamepad1.dpad_down) {
                   lifterPower(.5);
                }
            }
            lifterPower(0.0);

            idle();
        }
    }

    public void lifterPower(double power)
    {
        lifter.setPower(power);
        lifter2.setPower(power);
    }
}
