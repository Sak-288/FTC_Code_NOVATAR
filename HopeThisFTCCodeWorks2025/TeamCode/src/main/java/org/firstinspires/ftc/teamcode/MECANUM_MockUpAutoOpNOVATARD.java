package com.example.hopethisftccodereallydoesworkintothedeep2025.TeamCode.src.main.java.org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "MECANUM_MockUpAutoOpNOVATARD")
public class MECANUM_MockUpAutoOpNOVATARD extends LinearOpMode {

    // -----------------------------
    // 2) Class-level hardware
    // -----------------------------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, linearSlides;
    private CRServo intakeServo;
    private ColorSensor revColorSensor;

    // For color detection results
    public double red_detected;
    public double green_detected;
    public double blue_detected;
    public double light_intensity_detected;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(10, -60, 0));

        Pose2d startPose = new Pose2d(10, -60, 0);

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(10, -30))
                        .strafeTo(new Vector2d(10, -35))
                        //.lineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)), Math.toRadians(270))
                        .strafeTo(new Vector2d(47, -70))
                        //.lineToSplineHeading(new Pose2d(5, -35, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(5, -35, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(5, -30))
                        .strafeTo(new Vector2d(5, -35))
                        .strafeTo(new Vector2d(15, -35))
                        .splineToConstantHeading(new Vector2d(35, -12), Math.toRadians(90))
                        //.strafeTo(new Vector2d(20, -35))
                        .splineToConstantHeading(new Vector2d(47, -10), Math.toRadians(90))
                        .strafeTo(new Vector2d(47, -55))
                        .strafeTo(new Vector2d(47, -10))
                        .strafeTo(new Vector2d(57, -10))
                        .strafeTo(new Vector2d(57, -55))
                        .strafeTo(new Vector2d(57, -10))
                        .strafeTo(new Vector2d(67, -10))
                        .strafeTo(new Vector2d(67, -55))
                        .strafeTo(new Vector2d(47, -55))
                        .splineToSplineHeading(new Pose2d(47, -70, Math.toRadians(270)), Math.toRadians(270))
                        //.lineToSplineHeading(new Pose2d(0, -35, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(0, -30))
                        .strafeTo(new Vector2d(0, -35))
                        //.lineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)), Math.toRadians(270))
                        .strafeTo(new Vector2d(47, -70))
                        //.lineToSplineHeading(new Pose2d(-5, -35, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(-5, -35, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-5, -30))
                        .strafeTo(new Vector2d(-5, -35))
                        //.lineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(47, -50, Math.toRadians(270)), Math.toRadians(270))
                        .strafeTo(new Vector2d(47, -70))
                        //.lineToSplineHeading(new Pose2d(-10, -35, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(-10, -35, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-10, -30))
                        .strafeTo(new Vector2d(-10, -35))
                        .strafeTo(new Vector2d(60, -70))
                        .build());
    }

    // -----------------------------
    // 8) Color reading
    // -----------------------------
    public void getColor() {
        red_detected = revColorSensor.red();
        green_detected = revColorSensor.green();
        blue_detected = revColorSensor.blue();
        light_intensity_detected = revColorSensor.alpha();
    }

    // -----------------------------
    // 9) Stop routine
    // -----------------------------
    private void stop_robot() {
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        linearSlides.setPower(0.0);
        intakeServo.setPower(0.0);
    }
}
