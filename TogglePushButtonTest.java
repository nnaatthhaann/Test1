package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Ramroids 10318 on 12/27/2016.
 */

@TeleOp
@Disabled
public class TogglePushButtonTest extends LinearOpMode
{
    private DcMotor collector;

    @Override
    public void runOpMode() throws InterruptedException
    {
        collector = hardwareMap.dcMotor.get("collector");
        boolean toggle = false;

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                toggle = !toggle;

                if (toggle)
                {
                    collector.setPower(1.0);
                }

                //wait for button to be reset
                Thread.sleep(250);
            }

                if(toggle == false)
                {
                    collector.setPower(0.0);
                }

            while (gamepad1.right_bumper)
            {
                collector.setPower(-1.0);
            }

            idle();
        }
    }
}
