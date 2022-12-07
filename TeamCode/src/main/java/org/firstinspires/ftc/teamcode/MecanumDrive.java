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
    public static int lowerLimit = 0;
    public static int upperLimit = -6000;
//    private static int pickup = -20;
    public static int ground = 20;
    public static int low = 2550;
    public static int medium = 4120;
    public static int high = 5600;
    private double clawClose = 1.0;
    private double clawOpen = 0.0;
    //----------------------------------------------------------------------------------------------


    //Begin Linear Slide config
    public static int target = 0;
    public static int armUpSpeed = 15;
    public static int armDownSpeed = -15;
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

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        int position = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(-target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(-position > target){
            armMotor.setPower(-armDownSpeed);
        } else if (-position < target){
            armMotor.setPower(-armUpSpeed);
        } else {
            armMotor.setPower(0);
        }
        //------------------------------------------------------------------------------------------

        //Ground
        // TODO: Rishu daddy you forgot to add the time stuff nerd for the if statement
        // TODO: Man how hard is it nerd
        if(gamepad2.a){
           target = ground;
           claw.setPosition(clawOpen);
        }
        //Low
        if(gamepad2.b){
            target = low;
        }
        //Medium
        if(gamepad2.x){
            target = medium;
        }
        //High
        if(gamepad2.y){
            target = high;
        }

        //------------------------------------------------------------------------------------------

        if(gamepad2.right_trigger>.9 && -target >= upperLimit) {
//            armMotor.setPower(-1*armUpSpeed);
            target = target + armUpSpeed;
        } else if(gamepad2.left_trigger>.9 && -target <= lowerLimit){
//            armMotor.setPower(-1*armDownSpeed);
            target = target + armDownSpeed;
        }

        if (gamepad2.left_bumper) {
            claw.setPosition(clawOpen);
        }

        if (gamepad2.right_bumper) {
            claw.setPosition(clawClose);
        }
        //------------------------------------------------------------------------------------------

        telemetry.addData("Current Position: ", -position);
        telemetry.addData("Target Position: ", target);
        telemetry.addData("Lower Limit: ", -lowerLimit);
        telemetry.addData("Higher Limit: ", -upperLimit);
        //Added for Wheel Testing -- Yatharth -- 12/6/2022
        telemetry.addData("LF Motor Power", LeftFrontMotor.getPower());
        telemetry.addData("LF Motor Power", LeftFrontMotor.getVelocity());
        telemetry.addData("RF Motor Power", RightFrontMotor.getPower());
        telemetry.addData("RF Motor Power", RightFrontMotor.getVelocity());
        telemetry.addData("Difference", LeftFrontMotor.getVelocity()-RightFrontMotor.getVelocity());
    }
}

