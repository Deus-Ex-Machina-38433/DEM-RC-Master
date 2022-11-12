package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.*;

import java.util.Base64;

@Autonomous(name = "AutoBlue")
public class AutoBlue extends LinearOpMode {

    //TODO Set Values Below
    public static double leftReleased = 0.0;
    public static double rightReleased = 0.5;
    public static double leftClosed = 0.5;
    public static double rightClosed = 0.0;
    //End Servo Config

    //Preset Values
    public static int GLInner = -35;
    public static int GLOuter = 200;

    public static int LLInner = 0;
    public static int LLOuter = 350;

    public static int MLInner = -200;
    public static int MLOuter = 700;

    public static int HLInner = -175;
    public static int HLOuter = 900;

    //Begin Arm Values
    public static double outerSpeed = 3;
    public static double innerSpeed = 3;

    public static int outerUpperLimit = 600;
    public static int outerLowerLimit = 100;
    public static int innerUpperLimit = 300;
    public static int innerLowerLimit = -300;
    //End Arm Values

    //Begin Arm Outer Motor PID Declarations
    private PIDController controllerAMO;

    public static double pAMO = 0.04, iAMO = 0, dAMO =0.0002;
    public static double fAMO = 0.28;

    public static int targetAMOuter = 100;

    private final double ticks_in_degreeAMO = 1993.6 / 360.0; //Insert Value
    private final double ticks_per_RevAMO = 1993.6; //Insert Value

    private DcMotorEx AMOuter;
    //End Above

    //Begin Arm Inner Motor PID Declarations
    private PIDController controllerAMI;

    public static double pAMI = 0.04, iAMI = 0, dAMI =0.00015;
    public static double fAMI =.28;

    public static int targetAMInner = 0;

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

    Servo armRight, armLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        controllerAMI = new PIDController(pAMI, iAMI, dAMI);
        AMInner = hardwareMap.get(DcMotorEx.class, "AMInner");

        //Begin Arm Outer Motor PID Inits
        controllerAMO = new PIDController(pAMO, iAMO, dAMO);
        AMOuter = hardwareMap.get(DcMotorEx.class, "AMOuter");
        // End as Above

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRight = hardwareMap.servo.get("armRight");
        armLeft = hardwareMap.servo.get("armLeft");

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

        armRight = hardwareMap.servo.get("armRight");
        armLeft = hardwareMap.servo.get("armLeft");

        waitForStart();
        
        while(opModeIsActive){
             
        }
        
        sleep(1500);

        RightBackMotor.setPower(.6);
        RightFrontMotor.setPower(.6);
        LeftFrontMotor.setPower(.6);
        LeftBackMotor.setPower(.6);
        AMInner.setPower(.5);

        sleep(900);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        AMInner.setPower(.0);

    }

}
