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

    //----------------------------------------------------------------------------------------------
    // Linear Slide Pre-Programmed Heights
    public static int lowerLimit = -20;
    public static int upperLimit = -6000;
    private static int pickup = -20;
    public static int ground = -20;
    public static int low = -2850;
    public static int medium = -4525;
    public static int high = -5600;
    private double clawClose = 1.0;
    private double clawOpen = 0.0;
    //----------------------------------------------------------------------------------------------


    //Begin Linear Slide config
    public static double armUpSpeed = 0.85;
    public static double armDownSpeed = -0.45;
    int position = 0;
    
    //----------------------------------------------------------------------------------------------
    DcMotorEx LeftFrontMotor; // 0 - base
    DcMotorEx RightFrontMotor; // 1 - base
    DcMotorEx LeftBackMotor; // 2 - base
    DcMotorEx RightBackMotor; // 3 - basem
    DcMotorEx armMotor; // 0 - arm
    Servo claw; // 0 - arm
    //----------------------------------------------------------------------------------------------

    @Override
    public void init(){

        //------------------------------------------------------------------------------------------
        LeftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftFrontMotor");
        RightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightFrontMotor");
        LeftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftBackMotor");
        RightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightBackMotor");
        armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
        
        LeftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");
        //------------------------------------------------------------------------------------------
        int position = armMotor.getCurrentPosition();
//        slideController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){

        //------------------------------------------------------------------------------------------
        double speedMultiply;
        if(gamepad1.right_trigger > .75){
            speedMultiply = 1;
        } else if(gamepad1.left_trigger >.75){
            speedMultiply = .35;
        } else{
            speedMultiply = .69;
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
        //------------------------------------------------------------------------------------------
        //Ground
        /*
        if(gamepad2.a){
            if(position < -20){
                armMotor.setPower(-armDownSpeed);
            } else if (position >-20){
                armMotor.setPower(armDownSpeed);
            } else {
                armMotor.setPower(0);
            }
            // armMotor.setPower(0);
            // armMotor.setTargetPosition(-20);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // claw.setPosition(clawOpen);
        }
        //Low
        if(gamepad2.b){
            // armMotor.setTargetPosition(-2850);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setPower(0);
        }
        //Medium
        if(gamepad2.x){
            // armMotor.setPower(0);
            // armMotor.setTargetPosition(-4525);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setPower(-0.15);
        }
        //High
        if(gamepad2.y){
            // armMotor.setTargetPosition(-5600); 
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setPower(-0.2);
        }
        */
        //------------------------------------------------------------------------------------------

        int position = armMotor.getCurrentPosition();
        if(gamepad2.right_trigger>.9 && position >= upperLimit) {
            armMotor.setPower(-1*armUpSpeed);
        } else if(gamepad2.left_trigger>.9 && position <= lowerLimit){
            armMotor.setPower(-1*armDownSpeed);
        } else {
            armMotor.setPower(0);
        }

//        if(position == 5000){
//            claw.setPosition(1.0);
//        }

        //------------------------------------------------------------------------------------------
//        int armPos = armMotor.getCurrentPosition();
//
//        slideController.setPID(p, i, d);
//        double pid = slideController.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target/ticks_in_degreeAMO)) * f;
//
//        double power = (pid + ff);
//
//        armMotor.setPower(power);

        //------------------------------------------------------------------------------------------

        if (gamepad2.left_bumper) {
            claw.setPosition(clawOpen);
        }

        if (gamepad2.right_bumper) {
            claw.setPosition(clawClose);
        }
        //------------------------------------------------------------------------------------------

        telemetry.addData("Arm Position: ", position);
        telemetry.addData("Lower Limit: ", lowerLimit);
        telemetry.addData("Higher Limit: ", upperLimit);
//        telemetry.addData("pos:", armPos);
        //telemetry.addData("target", target);
    }
}

