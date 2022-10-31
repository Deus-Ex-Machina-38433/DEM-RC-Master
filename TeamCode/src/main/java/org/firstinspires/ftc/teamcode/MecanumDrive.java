package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.tensorflow.lite.Tensor;

@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    //Begin Arm Outer Motor PID Declarations
    private PIDController controller;

    public static double p = 0, i = 0, d =0;
    public static double f =0;

    private final double ticks_in_degree = 0 / 180.0; //Insert Value
    private final double ticks_per_Rev = 0; //Insert Value

    private DcMotorEx AMOuter;
    //End Above

    // Drivetrain Motors
    DcMotorEx RightFrontMotor; // Right Front Motor 1
    DcMotorEx LeftFrontMotor; // Left Front Motor 0
    DcMotorEx RightBackMotor; // Right Back Motor 3
    DcMotorEx LeftBackMotor; // Left Back Motor 2


    DcMotorEx AMInner; // Arm Motor Inner


    @Override
    public void init(){

        // Wheel Motors
        LeftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftFrontMotor");
        RightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightFrontMotor");
        LeftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftBackMotor");
        RightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightBackMotor");
        LeftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Misc Motors
//      AMInner = (DcMotorEx) hardwareMap.dcMotor.get("AMInner");

        //Begin Arm Outer Motor PID Inits
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AMOuter = hardwareMap.get(DcMotorEx.class, "AMOuter");
        // End as Above
    }

    @Override
    public void loop(){

        double speedMultiply;
        if(gamepad1.right_trigger > .75){
            speedMultiply = 1;
        } else if(gamepad1.left_trigger >.75){
            speedMultiply = .25;
        } else{
            speedMultiply = .69; //Nice
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
        if(gamepad1.a){
            LeftFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LeftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            AMOuter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Make better Reset thing, thatll mess with stuff
        }

        // Arm Outer Motor PID Code Begin
        int target = 0;// Set actual value or toss in a reset

        if(gamepad2.left_trigger > .9){
            target = target -100;
        } else if(gamepad2.right_trigger> .9){
            target = target +100;
        }
        controller.setPID(p, i, d);
        int armPos = AMOuter.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double velocity = (pid + ff) * ticks_per_Rev;

        AMOuter.setVelocity(velocity);

        telemetry.addData("pos:", armPos);
        telemetry.addData("target", target);
        // End As Above

        /*
        TODO:
        Chain Both arm motors together
        Camera reading stuff
        Some form of working autonomous (Time :(, Could do distance if thatll work,
        or like a run to position thing, doesn't need to be crazy)
        All PID tuning that can do
        Create Reset thing
        Servo Coding for claw
        Scratch Code for Nicks design
        Simple this file
        Convince for odometery pods cause nobody listen lmfao
        */
    }
}