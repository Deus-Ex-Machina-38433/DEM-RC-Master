package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.tensorflow.lite.Tensor;

@Config //This is needed to change variables marked as 'static' in Dashboard
@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    /*
    Relevant Stuff:
    inner: -520
    outer: 480
     */

    //Begin Servo Config
    public static double armOpen = 1.0;
    public static double armClose = 0.0;
    public static double clawPos = armClose;
    //End Servo Config

    //Begin Linear Slide Pre-Programmed Heights
    public static int ground = 0;
    public static int low = 50;
    public static int medium = 100;
    public static int high = 150;

    //Begin Linear Slide PID
    private PIDController controller;
    public static double p = 0.01, i = 0, d =0.0001;
    public static double f = 0.05;
    public static int target = 0;
    private final double ticks_in_degreeAMO = 1993.6 / 180.0;
    public static double armUpSpeed = 0.2;
    public static double armDownSpeed = -0.2;
    //End Linear Slide PID
    private double armOffset = 0;
    // Drivetrain Motors
    DcMotorEx LeftFrontMotor; // Left Front Motor 0
    DcMotorEx RightFrontMotor; // Right Front Motor 1
    DcMotorEx LeftBackMotor; // Left Back Motor 2
    DcMotorEx RightBackMotor; // Right Back Motor 3
    // End^^

    DcMotorEx armMotor;
    Servo claw;

    @Override
    public void init(){

        // Wheel Motors
        LeftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftFrontMotor");
        RightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightFrontMotor");
        LeftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftBackMotor");
        RightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightBackMotor");
        armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
        LeftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armOffset = armMotor.getCurrentPosition();

        // Misc Motors

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = hardwareMap.servo.get("claw");
    }

    @Override
    public void loop(){

        claw.setPosition(clawPos);

        double speedMultiply;
        if(gamepad1.right_trigger > .50){
            speedMultiply = 1;
        } else if(gamepad1.left_trigger >.50){
            speedMultiply = .25;
        } else{
            speedMultiply = .55;
        }


        double lateral = -gamepad1.left_stick_x;
        double longitudinal = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double wheelPower = Math.hypot(lateral, longitudinal);
        double stickAngleRadians = Math.atan2(longitudinal, lateral);
        stickAngleRadians = stickAngleRadians - Math.PI/4;
        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
        LeftFrontMotor.setPower(( -wheelPower * cosAngleRadians * factor + turn) * speedMultiply);
        RightFrontMotor.setPower(( -wheelPower * sinAngleRadians * factor - turn) * speedMultiply);
        LeftBackMotor.setPower(( -wheelPower * sinAngleRadians * factor + turn) * speedMultiply);
        RightBackMotor.setPower(( -wheelPower * cosAngleRadians * factor - turn) * speedMultiply);

        // LAST DITCH ROBOT RESET CODE
        // DO NOT ACTIVE UNLESS ABSOLUTELY NEEDED
        if(gamepad1.x){
            if(gamepad1.dpad_up){
            LeftFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LeftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            // Make better Reset thing, thatll mess with stuff
            }
        }

        //Begin Claw Servo Code :(

//		armRight.setPosition(leftInital);
//		armLeft.setPosition(rightInital);

//		To Close
//		if(gamepad2.dpad_up){
//            arm.setPosition(armClose);
//		}
//
//		// Release
//		if (gamepad2.dpad_down){
//            arm.setPosition(armOpen);
//		}

        if(gamepad2.a){
            armMotor.setTargetPosition((int) (ground-armOffset));
        }

        if(gamepad2.b){
            armMotor.setTargetPosition((int) (low-armOffset));
        }

        if(gamepad2.x){
            armMotor.setTargetPosition((int) (medium-armOffset));
        }

        if(gamepad2.y){
            armMotor.setTargetPosition((int) (high-armOffset));
        }

        if(gamepad2.dpad_up){
            target += 10;
        } else if(gamepad2.dpad_down){
            target -= 10;
        }

        //Linear Slide PID Start
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degreeAMO)) * f;

        double power = (pid + ff);

        armMotor.setPower(power);

        if (gamepad2.left_bumper) {
            clawPos = armClose;
        }

        if (gamepad2.right_bumper) {
            clawPos = armOpen;
        }


        telemetry.addData("pos:", armPos);
        telemetry.addData("target", target);
        telemetry.addData("clawPos", claw.getPosition());
        //Linear Slide PID End

        /*
        TODO:
        Chain Both arm motors together
        Camera reading stuff
        Some form of working autonomous (Time :(, Could do distance if thatll work,
        or like a run to position thing, doesn't need to be crazy)
        All PID tuning that can do
        Create Reset thing
        Servo Coding for claw (TEST)
        Scratch Code for Nicks design
        Simple this file
        Convince for odometery pods cause nobody listen lmfao
        Add height locks for scoring, Dk if will have enough driver practice before meet, to not use
        */
    }
}
