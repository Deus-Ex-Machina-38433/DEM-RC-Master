package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDforArmMotors extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d =0;
    public static double f =0;

    public static int target ;

    private final double ticks_in_degreeAMO = 1993.6 / 180.0; //Insert Value

    private DcMotorEx AMOuter;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AMOuter = hardwareMap.get(DcMotorEx.class, "AMOuter");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = AMOuter.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degreeAMO)) * f;

        double power = (pid + ff);

        AMOuter.setPower(power);

        telemetry.addData("pos:", armPos);
        telemetry.addData("target", target);
    }
}
