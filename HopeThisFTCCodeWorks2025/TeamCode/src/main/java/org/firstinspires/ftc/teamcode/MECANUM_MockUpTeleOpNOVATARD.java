package com.example.hopethisftccodereallydoesworkintothedeep2025.TeamCode.src.main.java.org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// FTC Hardware
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// REV IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MECANUM_MockUpTeleOpNOVATARD")
public class MECANUM_MockUpTeleOpNOVATARD extends OpMode {
    // --------------------------
    // 1) Make hardware fields
    // --------------------------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, linearSlides;
    private CRServo intakeServo;
    private IMU imu;

    // We’ll keep track of powers if we want to set them to zero in stop()
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower, linearPower, intakePower;

    // For convenience, we can store a variable for drive speed scaling
    private double DRIVE_TRAIN_POWER_FACTOR = 1.0;

    @Override
    public void init() {
        telemetry.addData("Status", "INIT SEQUENCE LAUNCHED");

        // --------------------------
        // 2) Initialize hardware
        // --------------------------
        frontLeftMotor  = hardwareMap.dcMotor.get("front_left_motor");
        backLeftMotor   = hardwareMap.dcMotor.get("back_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backRightMotor  = hardwareMap.dcMotor.get("back_right_motor");
        //linearSlides    = hardwareMap.get(DcMotor.class, "linear_slides");
        intakeServo     = hardwareMap.get(CRServo.class, "intake_servo");

        // Set run modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //linearSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linearSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set directions
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //linearSlides.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // --------------------------
        // 3) Initialize IMU
        // --------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        drive_function();
        linearslides_function();
        intake_function();
    }

    public void drive_function() {
        // Basic forward/back, strafe, turn
        double forwards_backwards = gamepad1.right_stick_y;
        double strafing = gamepad1.right_stick_x * 1.2;
        double turning = -gamepad1.left_stick_x;

        // Adjust speed if certain buttons are pressed
        if (gamepad1.x)      DRIVE_TRAIN_POWER_FACTOR = 0.25;
        else if (gamepad1.a) DRIVE_TRAIN_POWER_FACTOR = 0.50;
        else if (gamepad1.b) DRIVE_TRAIN_POWER_FACTOR = 0.75;
        else if (gamepad1.y) DRIVE_TRAIN_POWER_FACTOR = 1.00;

        // Reset IMU yaw if 'start' is pressed (optional)
        if (gamepad1.start) {
            imu.resetYaw();
        }

        // Field-centric logic:
        double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement vector opposite to the bot’s heading
        double rotatedX =  strafing * Math.cos(-botOrientation) - forwards_backwards * Math.sin(-botOrientation);
        double rotatedY =  strafing * Math.sin(-botOrientation) + forwards_backwards * Math.cos(-botOrientation);

        // We can call these "rotX" / "rotY"
        double rotX = rotatedX;
        double rotY = rotatedY;
        double rx   = turning;

        // Denominator ensures we don’t exceed 1.0 in any motor
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Calculate motor powers
        frontLeftPower  = (rotY + rotX + rx) / denominator;
        backLeftPower   = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower  = (rotY + rotX - rx) / denominator;

        // Apply scaled power
        frontLeftMotor.setPower(frontLeftPower  * DRIVE_TRAIN_POWER_FACTOR);
        backLeftMotor.setPower(backLeftPower    * DRIVE_TRAIN_POWER_FACTOR);
        frontRightMotor.setPower(frontRightPower * DRIVE_TRAIN_POWER_FACTOR);
        backRightMotor.setPower(backRightPower   * DRIVE_TRAIN_POWER_FACTOR);
    }

    public void linearslides_function() {
        // Use gamepad2’s right stick for slides
        linearPower = gamepad2.right_stick_y;
        //linearSlides.setPower(linearPower);

        // Simple telemetry
        if (linearPower > 0) {
            telemetry.addData("Linear Slides", "Extending");
        } else if (linearPower < 0) {
            telemetry.addData("Linear Slides", "Retracting");
        } else {
            telemetry.addData("Linear Slides", "Stopped");
        }
    }

    public void intake_function() {
        // The triggers are floats from 0.0 to 1.0
        if (gamepad2.right_trigger > 0.0) {
            intakePower = -1.0;
            telemetry.addData("Intake", "Running (Direction 1)");
        } else if (gamepad2.left_trigger > 0.0){
            intakePower = 1.0;
            telemetry.addData("Intake", "Running (Direction 2)");
        } else {
            intakePower = 0.0;
            telemetry.addData("Intake", "Stopped");
        }

        intakeServo.setPower(intakePower);
    }

    @Override
    public void stop() {
        // Just set everything to zero
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        //linearSlides.setPower(0);
        intakeServo.setPower(0);
    }
}
