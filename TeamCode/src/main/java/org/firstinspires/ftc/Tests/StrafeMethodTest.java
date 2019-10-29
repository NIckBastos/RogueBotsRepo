package org.firstinspires.ftc.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RogueBot;

@Autonomous(name = "StrafeMethodTest", group = "TeleOp")



public class StrafeMethodTest extends LinearOpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();
    private RogueBot robot = new RogueBot();   // Use a Pushbot's hardware

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double STRAFE_SPEED = 1.0;






    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        if(opModeIsActive()) {
            //encoderStrafe(STRAFE_SPEED, 10, 5);
            waitForStart();
            encoderMovement(STRAFE_SPEED, 50, 90, 2);
        }

    }


    public void encoderStrafe(double speed,
                              double strafeDistance,
                              double timeoutS) {


        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newRightFrontTarget = -robot.rightFrontMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newLeftBackTarget = -robot.leftBackMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {


            }
            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderMovement(double speed, double distance, double angleHeading,
                              double timeoutS) {

        angleHeading = Math.toRadians(angleHeading);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double frontLeftDistance = 0;
        double backLeftDistance = 0;
        double frontRightDistance = 0;
        double backRightDistance = 0;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double xDist;
        double yDist;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            xDist = distance*Math.cos(angleHeading);
            yDist = distance*Math.sin(angleHeading);

            if(yDist>= xDist){

                frontLeftDistance = Math.sqrt(yDist*yDist + xDist*xDist);
                frontRightDistance = Math.sqrt(yDist*yDist - xDist*xDist);
                backLeftDistance = Math.sqrt(yDist*yDist - xDist*xDist);
                backRightDistance = Math.sqrt(yDist*yDist + xDist*xDist);
            }else if(xDist> yDist){

                frontLeftDistance = Math.sqrt(xDist*xDist + yDist*yDist);
                frontRightDistance = Math.sqrt(xDist*xDist - yDist*yDist) * -1;
                backLeftDistance = Math.sqrt(xDist*xDist - yDist*yDist) * -1;
                backRightDistance = Math.sqrt(xDist*xDist + yDist*yDist);
            }

            frontLeftPower = (frontLeftDistance/distance);
            frontRightPower = (frontRightDistance/distance);
            backLeftPower = (backLeftDistance/distance);
            backRightPower = (backRightDistance/distance);


            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(frontLeftDistance * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(frontRightDistance * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(backLeftDistance * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(backRightDistance * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(frontLeftPower));
            robot.rightFrontMotor.setPower(Math.abs(frontRightPower));
            robot.leftBackMotor.setPower(Math.abs(backLeftPower));
            robot.rightBackMotor.setPower(Math.abs(backRightPower));





            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {


            }
            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}