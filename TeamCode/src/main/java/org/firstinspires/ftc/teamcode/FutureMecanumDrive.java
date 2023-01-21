package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.tensorflow.lite.Tensor;

@Config //This is needed to change variables marked as 'static' in Dashboard
@TeleOp(name = "FutureMecanumDrive", group = "Future")
@Disabled
public class FutureMecanumDrive extends OpMode {

    //----------------------------------------------------------------------------------------------

    // Linear Slide Pre-Programmed Heights
    public static int lowerLimit = 0;
    public static int upperLimit = -6000;
    public static int ground = 20;
    public static int low = 2550;
    public static int medium = 4120;
    public static int high = 5600;
    private double clawClose = 1.0;
    private double clawOpen = 0.0;
    private boolean isClawClosed = false;
    public static int LF = 1060;
    public static int RF = 1061;
    public static int LB = 1060;
    public static int RB = 1061;

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
    DcMotorEx RightBackMotor; // 3 - base
    DcMotorEx armMotor; // 0 - arm
    Servo claw; // 0 - arm

    //----------------------------------------------------------------------------------------------

    void _runToPosition(int LF, int RF, int LB, int RB) {
        LeftFrontMotor.setTargetPosition(LeftFrontMotor.getCurrentPosition() + LF);
        RightFrontMotor.setTargetPosition(RightFrontMotor.getCurrentPosition() + RF);
        LeftBackMotor.setTargetPosition(LeftBackMotor.getCurrentPosition() + LB);
        RightBackMotor.setTargetPosition(RightBackMotor.getCurrentPosition() + RB);
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontMotor.setPower(0.8);
        RightFrontMotor.setPower(0.8);
        LeftBackMotor.setPower(0.8);
        RightBackMotor.setPower(0.8);
    }

    //----------------------------------------------------------------------------------------------

    @Override
    public void init() {

        //------------------------------------------------------------------------------------------

        LeftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftFrontMotor");
        RightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightFrontMotor");
        LeftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftBackMotor");
        RightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightBackMotor");
        armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");

        //------------------------------------------------------------------------------------------

        int position = armMotor.getCurrentPosition();
//        slideController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        //------------------------------------------------------------------------------------------
        double speedMultiply;
        if (gamepad1.right_trigger > .75) {
            speedMultiply = 1;
        } else if (gamepad1.left_trigger > .75) {
            speedMultiply = .35;
        } else {
            speedMultiply = .69;
        }

        double lateral = -gamepad1.left_stick_x;
        double longitudinal = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        if (lateral <= 0.05 && longitudinal <= 0.05) {
            if (!LeftFrontMotor.isBusy()) {
                if (gamepad1.dpad_up) {
                    _runToPosition(LF, RF, LB, RB);
                } else if (gamepad1.dpad_down) {
                    _runToPosition(-LF, -RF, -LB, -RB);
                } else if (gamepad1.dpad_left) {
                    _runToPosition(-LF, RF, LB, -RB);
                } else if (gamepad1.dpad_right) {
                    _runToPosition(LF, -RF, -LB, RB);
                }
            }
        } else {
            LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double wheelPower = Math.hypot(lateral, longitudinal);
        double stickAngleRadians = Math.atan2(longitudinal, lateral);
        stickAngleRadians = stickAngleRadians - Math.PI / 4;
        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
        LeftFrontMotor.setPower((-wheelPower * cosAngleRadians * factor + turn) * speedMultiply);
        RightFrontMotor.setPower((-wheelPower * sinAngleRadians * factor - turn) * speedMultiply);
        LeftBackMotor.setPower((-wheelPower * sinAngleRadians * factor + turn) * speedMultiply);
        RightBackMotor.setPower((-wheelPower * cosAngleRadians * factor - turn) * speedMultiply);
        //------------------------------------------------------------------------------------------
        int position = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(-target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (-position > target) {
            armMotor.setPower(-armDownSpeed);
        } else if (-position < target) {
            armMotor.setPower(-armUpSpeed);
        } else {
            armMotor.setPower(0);
        }
        //------------------------------------------------------------------------------------------

        //Ground
        // TODO: Rishu daddy you forgot to add the time stuff nerd for the if statement
        // TODO: Man sha' yo mou'
        //
        //if !claw.getPosition() == 1{
        //wait(250)
        //else{
        //  claw.setPosition(clawClose);
        //  wait(250)
        //}
        //}


        if (gamepad2.a) {
            isClawClosed = false;
            target = ground;
            claw.setPosition(clawOpen);
        }
        //Low
        if (gamepad2.b) {
//            claw.setPosition(clawClose);
//            sleep(100);
            target = low;
        }
        //Medium
        if (gamepad2.x) {
//            claw.setPosition(clawClose);
//            sleep(100);
            target = medium;
        }
        //High
        if (gamepad2.y) {
//            claw.setPosition(clawClose);
//            sleep(100);

            target = high;
        }

        //------------------------------------------------------------------------------------------

        if (gamepad2.right_trigger > .9 && -target >= upperLimit) {
//            armMotor.setPower(-1*armUpSpeed);
            target = target + armUpSpeed;
        } else if (gamepad2.left_trigger > .9 && -target <= lowerLimit) {
//            armMotor.setPower(-1*armDownSpeed);
            target = target + armDownSpeed;
        }
// This program must make the robot work
        if (gamepad2.left_bumper) {
            isClawClosed = false;
            claw.setPosition(clawOpen);
        }

        if (gamepad2.right_bumper) {
            isClawClosed = true;
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
        telemetry.addData("Difference", LeftFrontMotor.getVelocity() - RightFrontMotor.getVelocity());
    }
}