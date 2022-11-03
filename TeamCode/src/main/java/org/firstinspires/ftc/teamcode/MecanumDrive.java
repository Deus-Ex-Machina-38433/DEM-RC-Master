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
    private PIDController controllerAMO;

    public static double pAMO = 0, iAMO = 0, dAMO =0;
    public static double fAMO = 0.25;

    private final double ticks_in_degreeAMO = 1993.6 / 360.0; //Insert Value
    private final double ticks_per_RevAMO = 1993.6; //Insert Value

    private DcMotorEx AMOuter;
    //End Above
    
    //Begin Arm Inner Motor PID Declarations
    private PIDController controllerAMI;

    public static double pAMI = 0, iAMI = 0, dAMI =0;
    public static double fAMI =.3;

    private final double ticks_in_degreeAMI = 751.8 / 360.0; //Insert Value
    private final double ticks_per_RevAMI = 751.8; //Insert Value

    private DcMotorEx AMInner;
    //End As Above

    // Drivetrain Motors
    DcMotorEx RightFrontMotor; // Right Front Motor 1
    DcMotorEx LeftFrontMotor; // Left Front Motor 0
    DcMotorEx RightBackMotor; // Right Back Motor 3
    DcMotorEx LeftBackMotor; // Left Back Motor 2
    // End^^
    
//    Servo armRight, armLeft;

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
        
        //Begin Inner Motor PID Inits
        controllerAMI = new PIDController(pAMI, iAMI, dAMI);
        AMInner = hardwareMap.get(DcMotorEx.class, "AMInner");

        //Begin Arm Outer Motor PID Inits
        controllerAMO = new PIDController(pAMO, iAMO, dAMO);
        AMOuter = hardwareMap.get(DcMotorEx.class, "AMOuter");
        // End as Above
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
//        armRight = hardwareMap.servo.get("armRight");
//        armLeft = hardwareMap.servo.get("armLeft");
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
            AMInner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Make better Reset thing, thatll mess with stuff
        }

        // Arm Outer Motor PID Code Begin
        int targetAMOuter = 0; // Set actual value or toss in a reset
        controllerAMO.setPID(pAMO, iAMO, dAMO);
        int armPosAMO = AMOuter.getCurrentPosition();
        double pidAMO = controllerAMO.calculate(armPosAMO, targetAMOuter);
        double ffAMO = Math.cos(Math.toRadians(targetAMOuter/ticks_in_degreeAMO)) * fAMO;
        //double PowerAMO = (pidAMO + ffAMO);

        if(gamepad2.right_bumper){
            AMOuter.setPower(.6);
        } else if(gamepad2.left_bumper){
            AMOuter.setPower(-.1);
        } else{
            AMOuter.setPower(ffAMO);
        }

        int targetAMInner = 0; // Set ctual value or toss in a reset
        controllerAMI.setPID(pAMI, iAMI, dAMI);
        int armPosAMI = AMInner.getCurrentPosition();
        double pidAMI = controllerAMI.calculate(armPosAMI, targetAMInner);
        double ffAMI = Math.cos(Math.toRadians(targetAMInner/ticks_in_degreeAMI)) * fAMI;
        double velocityAMI = (pidAMI + ffAMI);
        telemetry.addData("posInner:", armPosAMI);
        telemetry.addData("targetAMInner:", targetAMInner);

        if(gamepad2.right_trigger > .9){
            AMInner.setPower(-.5);
        } else if(gamepad2.left_trigger >.9){
            AMInner.setPower(.6);
        } else{
            AMInner.setPower(ffAMI);
        }


        telemetry.addData("posOuter:", armPosAMO);
        telemetry.addData("targetAMOuter:", targetAMOuter);
        // End As Above
        
        
        
        //Arm Height controls non-manual, Add in corresponding values for other part of the ARM
        /*
        if(gamepad2.a){
            targetAMOuter = (int) 750.0;//Insert Ideal Value Highest
            targetAMInner = (int) 0.0;//^^
        } else if(gamepad2.b) {
            targetAMOuter = (int) 1.0;//Middle Value
            targetAMInner = (int) 1.0;//^^
        } else if(gamepad2.x) {
            targetAMOuter = (int) 2.0;//Low value
            targetAMInner = (int) 2.0;//^^
        } else {
            if(gamepad2.left_trigger > .9){
                targetAMOuter = targetAMOuter -100;
                targetAMInner = (int) (targetAMInner + 0.0);//Insert Value
            } else if(gamepad2.right_trigger> .9){
                targetAMOuter = targetAMOuter +100;
                targetAMInner = (int) (targetAMInner - 0.0);//Insert Value
            }
        }
        */
        //End^^
        
        //Begin Claw Servo Code :(
		
        double leftInital = 0.0; //Set Value
		double rightInital = 0.0;//^^
		
//		armRight.setPosition(leftInital);
//		armLeft.setPosition(rightInital);
        
		//To Close
		if(gamepad2.right_bumper){
//			armRight.setPosition(0.0); //SetValue
//			armLeft.setPosition(0.0); //SetValue
		} 
		
		// Release
		if (gamepad2.left_bumper){
//			armRight.setPosition(rightInital);
//			armLeft.setPosition(leftInital);
		}
        
        
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
