package org.firstinspires.ftc.teamcode;

import android.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.tensorflow.lite.Tensor;

@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    //Variable Declarations Arm Outer Motor
    public double lastErrorAMO = 0;
    public double integralSumAMO = 0;

    public static double KpAMO = 0.0;
    public static double KiAMO = 0.0;
    public static double KdAMO = 0.0;
    DcMotorEx AMOuter; // Arm Motor Outer

    // Wheel Motors
    DcMotorEx RightFrontMotor; // Right Front Motor
    DcMotorEx LeftFrontMotor; // Left Front Motor
    DcMotorEx RightBackMotor; // Right Back Motor
    DcMotorEx LeftBackMotor; // Left Back Motor
    // Misc Motors
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
        AMOuter = (DcMotorEx) hardwareMap.dcMotor.get("AMOuter");
    }

    @Override
    public void loop(){

        AMOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();

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

        if(gamepad2.left_trigger > .9){
            AMOuter.setVelocity(-100);
        } else if(gamepad2.right_trigger> .9){
            AMOuter.setVelocity(100);
        }

        if(gamepad1.a){
            LeftFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LeftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            AMOuter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        
    }

    public void PIDforMotor () {

    }
}
//  Find Ticks Per Motor Rotation and Multiply the internal Power Value by that to Switch it to Velocity
//  Easy to fix if it doesnt work dont have motor inverts for our current robot