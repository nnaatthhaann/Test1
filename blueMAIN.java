/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.webkit.JavascriptInterface;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "BLUE")  // @Autonomous(...) is the other common choice
public class blueMAIN extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //dcmotors
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor collector;
    private DcMotor shooter;
    private DcMotor lifter;
    private DcMotor lifter2;

    //servos
    private Servo beacon;
    private Servo lift;

    //sensors
    private ModernRoboticsAnalogOpticalDistanceSensor floorODS;
    private ModernRoboticsAnalogOpticalDistanceSensor wallODS;
    private ModernRoboticsI2cColorSensor color;
    private ModernRoboticsI2cRangeSensor range;
    private ModernRoboticsI2cGyro gyro;
    private EventLoopManager eventLoopManager1;

    @Override   //INITIALIZE
    public void init() {
        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        lifter = hardwareMap.dcMotor.get("lifter");
        lifter2 = hardwareMap.dcMotor.get("lifter2");

        beacon = hardwareMap.servo.get("beacon");
        lift = hardwareMap.servo.get("lift");

        floorODS = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods1");
        wallODS = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");
        color = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        color.enableLed(false);         //set color sensor to passive mode

        int zAccumulated;
        int target = 0;
        double TURN_POWER = .2;
        double POWER = .5;

        //gyro sensor calib
        gyro.calibrate();
        telemetry.addData("Gyro Sensor:", "Calibrating... Keep robot still");
        while (gyro.isCalibrating()) {
        }
        telemetry.addData("Gyro Sensor:", "Done");
        telemetry.addData("Status:", "Initialized");
    }

    //establishes default magic number for drive power
    double POWER = .5;

    @Override
    public void init_loop() {
    }

    @Override   //AUTONOMOUS
    public void start() {
        telemetry.addData("Status:", "Autonomous");

        DriveForwardTime(.5, 500);
        TurnDegrees(30);
        while (range.getDistance(DistanceUnit.CM) > 5)
            DriveForward(1);
        pause();
        while (gyro.getIntegratedZValue() > 0){
            front_right.setPower(.5);
            back_right.setPower(.5);
            front_left.setPower(.1);
            back_left.setPower(.1);
        }
        //back up to line
        //check beacon color
        //move or push beacon
        //follow wall to next line
        //check beaconcolor
        //move or push beacon
        //turn away from wall
        //approach hoop
        //shoot 2 balls




        //first beacon
        DriveForward(.4);
        while (floorODS.getRawLightDetected() < 800) {
        }
        pause();
        DriveForwardTime(POWER, 400);

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

        DriveForwardTime(1, 3500);

        /*
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
        */
    }

    @Override   //TELEOP - complete
    public void loop() {
        telemetry.addData("Status:", "TeleOp");
        telemetry.update();

        boolean toggle = false;

        front_left.setPower(-gamepad1.left_stick_y);
        back_left.setPower(-gamepad1.left_stick_y);

        front_right.setPower(-gamepad1.right_stick_y);
        back_right.setPower(-gamepad1.right_stick_y);

        //left and right trigger
        double mCalib = 0.5;
        front_left.setPower((gamepad1.left_trigger * mCalib) + (-gamepad1.left_stick_y));
        back_left.setPower((-gamepad1.left_trigger) + (-gamepad1.left_stick_y));
        front_right.setPower((-gamepad1.left_trigger * mCalib) + (-gamepad1.right_stick_y));
        back_right.setPower((gamepad1.left_trigger) + (-gamepad1.right_stick_y));

        front_left.setPower((-gamepad1.right_trigger * mCalib) + (-gamepad1.left_stick_y));
        back_left.setPower((gamepad1.right_trigger) + (-gamepad1.left_stick_y));
        front_right.setPower((gamepad1.right_trigger * mCalib) + (-gamepad1.right_stick_y));
        back_right.setPower((-gamepad1.right_trigger) + (-gamepad1.right_stick_y));

        if (gamepad1.right_bumper) {
            double power = 0.5;
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(-power);
            back_right.setPower(-power);
        }
        if (gamepad1.left_bumper) {
            double power = -0.5;
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(-power);
            back_right.setPower(-power);
        }

        //collector toggle
        if (gamepad1.a) {
            //toggle state of boolean value
            toggle = !toggle;
        }
        //turns on collector if value is true
        if (toggle) {
            collector.setPower(1.0);
        } else {
            collector.setPower(0);
        }

        //spits out ball
        while (gamepad1.b) {
            collector.setPower(-1.0);
        }

        while (gamepad1.x) {
            shooter.setPower(1.0);
        }
        while (gamepad1.y) {
            shooter.setPower(-1.0);
        }
        shooter.setPower(0.0);

        //Cap Ball Lifter
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            while (gamepad1.dpad_up) {
                lifterPower(-1);
            }
            while (gamepad1.dpad_down) {
                lifterPower(.5);
            }
        }
        lifterPower(0.0);
    }

    @Override   //complete
    public void stop() {
    }

    //USER DEFINED METHODS:

    //Autonomous Methods
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
/*
    private void TurnRightTime(double power, long time) {
        runtime.reset();
        runtime.startTime();
        //default right
        do {
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(-power);
            back_right.setPower(-power);
        } while (runtime.time() < time);
        pause();
    }
*/
    private void pause() {
        DriveForward(0);
        while (runtime.time() < 400) {
        }
    }

    private void TurnDegrees(int target) {
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

    /*
    private void Drive(double maxpower, long time) throws InterruptedException {
        double out;
        int t = 0;        //t = runtime

        //Decceleration of motors
        if (maxpower > 0) //if power is pos
        {
            //Acceleration of the 4 drive motors
            for (double x = .1; x <= maxpower; x += .1) {
                out = x;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);


           //Every 50 milis add one value to variable 'time'
           //Thus, resulting 'time' is length of runtime in sets of 50 milis

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
        } else        //if power is neg
        {
            //Acceleration of the 4 drive motors
            for (double x = -.1; x >= maxpower; x -= .1) {
                out = x;

                front_left.setPower(out);
                back_left.setPower(out);
                front_right.setPower(out);
                back_right.setPower(out);
                Thread.sleep(50);


           //Every 50 milis add one value to variable 'time'
           //Thus, resulting 'time' is length of runtime in sets of 50 milis

                t = t + 1;
            }

            DriveForwardTime(maxpower, time - 2 * (50 * t));

            //Decceleration of motors
            for (double z = maxpower; z <= -.05; z += .05) {
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
    */

    private void ShootTime(long time) {
        runtime.reset();
        runtime.startTime();
        do {
            shooter.setPower(1);
        } while (runtime.time() < time);
    }

    //TeleOp Methods
    private void lifterPower(double power) {
        lifter.setPower(power);
        lifter2.setPower(power);
    }
}