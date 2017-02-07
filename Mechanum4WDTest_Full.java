package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ramroids 10318 on 12/30/2016.
 */
@TeleOp (name = "Mechanum4WD Test")
@Disabled
public class Mechanum4WDTest_Full extends LinearOpMode
{
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    @Override
    public void runOpMode() throws InterruptedException
    {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {

            while (gamepad1.right_stick_y != 0 && gamepad1.right_stick_x != 0)
            {
                double y = gamepad1.right_stick_y;
                double x = gamepad1.right_stick_x;
                double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

                while (x > 0) {
                    while (y / x > 1) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * (((Math.atan(y / x) * deg) - 45) / -90));
                        back_left.setPower(power * (((Math.atan(y / x) * deg) - 45) / -90));
                        //set 1 done
                        front_left.setPower(power * ((((Math.atan(y / x) * deg) + 45) / -90) + 2));
                        back_right.setPower(power * ((((Math.atan(y / x) * deg) + 45) / -90) + 2));
                    }
                    while ((y / x >= -1) && (y / x <= 1)) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * (((Math.atan(y / x) * deg) - 45) / 90));
                        back_left.setPower(power * (((Math.atan(y / x) * deg) - 45) / 90));
                        //set 1 done
                        front_left.setPower(power * (((Math.atan(y / x) * deg) + 45) / 90));
                        back_right.setPower(power * (((Math.atan(y / x) * deg) + 45) / 90));
                    }
                    while (y / x < -1) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * ((((Math.atan(y / x) * deg) - 45) / -90) - 2));
                        back_left.setPower(power * ((((Math.atan(y / x) * deg) - 45) / -90) - 2));
                        //set 1 done
                        front_left.setPower(power * (((Math.atan(y / x) * deg) + 45) / -90));
                        back_right.setPower(power * (((Math.atan(y / x) * deg) + 45) / -90));
                    }
                }

                while (x < 0) {
                    //reverse of positive x?
                    while (y / x < -1) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * (((Math.atan(y / x) * deg) - 45)/ 90) + 2);
                        back_left.setPower(power * (((Math.atan(y / x) * deg) - 45)/ 90) + 2);
                        //set 1 done
                        front_left.setPower(power * ((Math.atan(y / x) * deg) + 45)/ -90);
                        back_right.setPower(power * ((Math.atan(y / x) * deg) + 45)/ -90);
                    }

                    while ((y / x > -1) && (y / x < 1)) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * ((Math.atan(y / x) * deg) - 45)/ -90);
                        front_right.setPower(power * ((Math.atan(y / x) * deg) - 45)/ -90);
                        //set 1 done
                        front_right.setPower(power * ((Math.atan(y / x) * deg) + 45)/ -90);
                        front_right.setPower(power * ((Math.atan(y / x) * deg) + 45)/ -90);
                    }

                    while (y / x > 1) {
                        updatePower();
                        //set 2 done
                        front_right.setPower(power * ((Math.atan(y / x) * deg) - 45)/ 90);
                        front_right.setPower(power * ((Math.atan(y / x) * deg) - 45)/ 90);
                        //set 1 done
                        front_right.setPower(power * (((Math.atan(y / x) * deg) - 45)/ -90) - 2);
                        front_right.setPower(power * (((Math.atan(y / x) * deg) - 45)/ -90) - 2);
                    }
                }

                while (gamepad1.left_stick_x == 0) {
                    front_right.setPower(gamepad1.right_stick_y);
                    back_right.setPower(gamepad1.right_stick_y);
                    front_left.setPower(gamepad1.right_stick_y);
                    back_left.setPower(gamepad1.right_stick_y);
                }

            }

            while (gamepad1.left_stick_x != 0) {
                front_left.setPower(gamepad1.left_stick_x);
                back_left.setPower(gamepad1.left_stick_x);
                front_right.setPower(-gamepad1.left_stick_x);
                back_right.setPower(-gamepad1.left_stick_x);
            }

            idle();
        }
    }

    double deg = 180 / Math.PI;

    public void updatePower() {
        double y = -gamepad1.right_stick_y;
        double x = -gamepad1.right_stick_x;

        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
}
