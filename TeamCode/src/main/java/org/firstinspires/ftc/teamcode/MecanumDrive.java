package org.firstinspires.ftc.teamcode;

import android.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    // Wheel Motors
    DcMotorEx RFMotor; // Right Front Motor
    DcMotorEx LFMotor; // Left Front Motor
    DcMotorEx RBMotor; // Right Back Motor
    DcMotorEx LBMotor; // Left Back Motor
    // Misc Motors
    DcMotorEx AMInner; // Arm Motor Inner
    DcMotorEx AMOuter; // Arm Motor Outer

    @Override
    public void init(){

        // Wheel Motors
        LFMotor = (DcMotorEx) hardwareMap.dcMotor.get("LFMotor");
        RFMotor = (DcMotorEx) hardwareMap.dcMotor.get("RFMotor");
        LBMotor = (DcMotorEx) hardwareMap.dcMotor.get("LBMotor");
        RBMotor = (DcMotorEx) hardwareMap.dcMotor.get("RBMotor");
        LFMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LBMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Misc Motors
        AMInner = (DcMotorEx) hardwareMap.dcMotor.get("AMInner");
        AMOuter = (DcMotorEx) hardwareMap.dcMotor.get("AMOuter");
    }

    @Override
    public void loop(){

        double speedMultiply;
        if(gamepad1.right_trigger > .75){
            speedMultiply = 1;
        } else if(gamepad1.left_trigger >.75){
            speedMultiply = .25;
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
        LFMotor.setPower(( -wheelPower * cosAngleRadians * factor + turn) * speedMultiply);
        RFMotor.setPower(( -wheelPower * sinAngleRadians * factor - turn) * speedMultiply);
        LBMotor.setPower(( -wheelPower * sinAngleRadians * factor + turn) * speedMultiply);
        RBMotor.setPower(( -wheelPower * cosAngleRadians * factor - turn) * speedMultiply);

        if(gamepad1.a){
            LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
//  Find Ticks Per Motor Rotation and Multiply the internal Power Value by that to Switch it to Velocity
//  Easy to fix if it doesnt work dont have motor inverts for our current robot