package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.Locale;

//import static org.firstinspires.ftc.teamcode.RobotMovement.followCurve;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="HookTest", group="Pushbot")
//@Disabled
public class HookTest extends LinearOpMode {

    // The IMU sensor object

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* Declare OpMode members. */
    private RogueBot robot = new RogueBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    private static final double COUNTS_PER_MOTOR_REV = 134;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.3;
    static final double Lift_Speed = 0.4;
    int leftFrontCurrentPosition = 0;
    int rightFrontCurrentPositon = 0;
    int leftBackCurrentPosition = 0;
    int rightBackCurrentPosition = 0;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AWmj6yz/////AAABmfWffA2XHkI1inDJh0chZecmL6yY3yRHSKsD9+mfYbQVgp2U7OpD5dl+bEAsi9S0X250X0V5q8k+euywF8obWxxwH/R2vhgOECD7x/i2I22NgD8mXR83z9riqwOdPuTEOyKXZsYLLPExzxzYPn+X+J1w7qarT/5B32XXq5OfrGDr4Ut3EimQWePJavRb6Drhiki7rnbJz6Q1DETY51gyvpy07jFSxImkNnMEcYvGMcOHZU79s4eSPmaoCx41ftp/v48AqGE2oEOVWo1WD15MuBG7i11qVpYoM4KKFf13DS5hABRQpRSYyj5HVC67kwGvub2tUruOwVxZOxoBax+ETFpfNvHFbz2so/yipxWNXUEG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();

        // This should deploy the intake
        encoderLift(0.6,4,2);
        rotateServo();
        encoderLift(0.6,-4,2);

        // This will latch to the foundation
        hookDown();
        hookUp();


    }


    public void hookDown(){
        robot.hookServo_1.setPosition(1);//up
        robot.hookServo_2.setPosition(-1);//up

        sleep(1000);
        telemetry.addData("down",' ');
        telemetry.update();
    }

    public void hookUp(){
        robot.hookServo_1.setPosition(-1);//up
        robot.hookServo_2.setPosition(1);//up
        
        telemetry.addData("up",' ');
        telemetry.update();
        sleep(1000);

        robot.hookServo_1.setPosition(0);//up
        robot.hookServo_2.setPosition(0);//up
        telemetry.addData("up",' ');
        telemetry.update();
    }

    public void rotateServo(){
        robot.rotateServo.setPosition(-1);
        telemetry.addData("rotate","");
        sleep(1000);
    }

    public void encoderLift(double speed, double height, double timeoutS){
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Reset Encoders
            robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.liftMotorLeft.getCurrentPosition() + (int)(height * COUNTS_PER_INCH);
            newRightTarget = robot.liftMotorRight.getCurrentPosition() + (int)(height*COUNTS_PER_INCH);

            // Assign new raget position
            robot.liftMotorLeft.setTargetPosition(newLeftTarget);
            robot.liftMotorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotorLeft.setPower(Math.abs(speed));
            robot.liftMotorRight.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotorLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.liftMotorLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotorLeft.setPower(0);
            robot.liftMotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


}