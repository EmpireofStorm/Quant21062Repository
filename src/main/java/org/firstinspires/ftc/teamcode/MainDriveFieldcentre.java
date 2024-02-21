package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class MainDriveFieldcentre extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor lift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor lift2 = hardwareMap.dcMotor.get("lift2");
        DcMotor lift3 = hardwareMap.dcMotor.get("lift3");
        DcMotor motorintake = hardwareMap.dcMotor.get("intake");
        Servo servo1 = hardwareMap.servo.get("claw1");
        Servo servo2 = hardwareMap.servo.get("claw2");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.'
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//                  RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                  RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                    new Orientation(
                            AxesReference.INTRINSIC,
                            AxesOrder.ZYX,
                            AngleUnit.DEGREES,
                            90,
                            -45,
                            /*-90*/ 90,
                            0
                    )));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if(gamepad1.left_bumper){
                lift1.setPower(-1);
                lift2.setPower(1);
            } else if(gamepad1.left_trigger > 0.3){
                lift1.setPower(1);
                lift2.setPower(-1);
            } else {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if(gamepad2.right_bumper){
                motorintake.setPower(1);
            } else  if(gamepad2.right_trigger> 0.3){
                motorintake.setPower(-1);
            } else motorintake.setPower(0);
            if(gamepad1.right_bumper){
                servo1.setPosition(-1);
                servo2.setPosition(1);
            } else if(gamepad1.right_trigger>0.3){
                servo1.setPosition(1);
                servo2.setPosition(-1);
            }//perpendicular 5.906x 3.937y parallel 7.48y 1.969x
            if(gamepad2.left_bumper){
                lift3.setPower(1);
            } else if(gamepad2.left_trigger > 0.3){
                lift3.setPower(-1);
        } else lift3.setPower(0);

            telemetry.addData("leftRear: ", backLeftMotor.getPower());
            telemetry.addData("leftFront: ", frontLeftMotor.getPower());
            telemetry.addData("rightRear: ", backRightMotor.getPower());
            telemetry.addData("rightFront: ", frontRightMotor.getPower());
            telemetry.addData("lift1: ", lift1.getPower());
            telemetry.addData("lift2: ", lift2.getPower());
            telemetry.addData("lift3: ", lift3.getPower());
            telemetry.update();


    }}}