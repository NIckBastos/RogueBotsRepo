package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainTeleOpMode", group = "TeleOp")

public class MainTeleOpMode extends OpMode {

    //Creating variables
    private ElapsedTime runtime = new ElapsedTime();

    double frontLeftPower,backLeftPower, frontRightPower, backRightPower;
    double drive;   // Power for forward and back motion
    double strafe;  // Power for left and right motion
    double rotate;  // Power for rotating the robot
    double startingAngle =-90;
    double currentAngle = 0;

    final private static double JOYSTICK_DEADBAND = 0.1;

    //Encoder Ticks Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        (WHEEL_DIAMETER_INCHES * 3.1415);


    double motorMovementMin = 0.0;
    double motorMovementMax = 0.0;


    private RogueBot robot;

    @Override
    public void init() {

        robot = new RogueBot();
        robot.init(hardwareMap);
//        robot.scoopMotor.setTargetPosition(0);
    }

    //Code that resets the elapsed time once the driver hits play
    @Override
    public void start() {
        runtime.reset();
    }




    public void loop() {

        //Finding the value of each joystick
        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        // Joystick Deadband
        if (Math.abs(drive) < JOYSTICK_DEADBAND) {drive = 0;}
        if (Math.abs(strafe) < JOYSTICK_DEADBAND) {strafe = 0;}


        //Finding the power to assign for each motor
        frontLeftPower = drive + strafe + rotate;
        backLeftPower = drive - strafe + rotate;
        frontRightPower = drive + strafe - rotate;
        backRightPower = drive - strafe - rotate;

        // Setting the power to each motor
        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.leftBackMotor.setPower(backLeftPower);
        robot.rightFrontMotor.setPower(frontRightPower);
        robot.rightBackMotor.setPower(backRightPower);






//        // If dpad in gampepad1 is pressed turn the arm
//        if(!gamepad2.dpad_down && gamepad2.dpad_up){
//            robot.armMotor.setPower(.7);
//            robot.armMotor.setTargetPosition(-270);
//        }else if (gamepad2.dpad_down && !gamepad2.dpad_up){
//            robot.armMotor.setPower(.7);
//            robot.armMotor.setTargetPosition(270);
//
//        }else{
//            robot.armMotor.setPower(0);
//        }


        //Setting the power of the intake servo to 1 // Intake
        if (gamepad2.right_bumper) {
            robot.intakeServo_2.setPower(1);
            robot.intakeServo_1.setPower(-1);
            robot.intakeServo_3.setPower(1);
            robot.intakeServo_4.setPower(-1);

        } else if (gamepad2.left_bumper) { // Setting the power of the intake servo to -1 // Output
            robot.intakeServo_2.setPower(-1);
            robot.intakeServo_1.setPower(1);
            robot.intakeServo_3.setPower(-1);
            robot.intakeServo_4.setPower(1);
        }else{ // Setting the power of the intake servo to 0 // Zero behaviour
            robot.intakeServo_1.setPower(0);
            robot.intakeServo_2.setPower(-0.05);
            robot.intakeServo_3.setPower(0);
            robot.intakeServo_4.setPower(0);
        }


        if(gamepad2.y){
            //currentAngle = currentAngle+5;
            robot.flipServo_1.setPosition(-90);
            robot.flipServo_2.setPosition(-90);

        } else if(gamepad2.b){
            //currentAngle = currentAngle-5;
            robot.flipServo_1.setPosition(-270);
            robot.flipServo_2.setPosition(270);
        }







        //Telemetry is not used to control the robot, it is purely to help debug by showing
        //Information on the phone
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Value of joystick = ", "Drive (%.2f), Strafe (%.2f)", gamepad2.right_stick_y, gamepad2.right_stick_x);

        // Display all motor power
        telemetry.addData("Motor Power ", "FL (%.2f), FR (%.2f)", robot.leftFrontMotor.getPower() , robot.rightFrontMotor.getPower());
        telemetry.addData("Motor Power ", "BL (%.2f), BR (%.2f)" , robot.leftBackMotor.getPower(), robot.leftBackMotor.getPower());



        // Display flip servo position
        telemetry.addData("Servo 1 position" , robot.flipServo_1.getPosition());
        telemetry.addData("Servo 2 position", robot.flipServo_2.getPosition());
        telemetry.update();

    }
}