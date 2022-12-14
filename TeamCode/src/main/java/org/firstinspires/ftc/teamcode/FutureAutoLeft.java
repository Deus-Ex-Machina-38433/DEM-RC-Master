/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous
public class FutureAutoLeft extends LinearOpMode
{
    // vv April Tag Stuff DO NOT TOUCH vv
    double PowerFactor = 12.06/13.31;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    //TODO: You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.508;

    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;
    // ^^ April Tag Stuff DO NOT TOUCH ^^

    // vv Run to Position config vv (how much ticks needed to cross exactly one mat)
    public static int all = 1000;
    public static boolean override = false; // Enable to use variables underneath
    public static int LF = 1000;
    public static int RF = 1000;
    public static int LB = 1000;
    public static int RB = 1000;
    // ^^ Run to Position config ^^

    DcMotorEx LeftFrontMotor; // 0 - base
    DcMotorEx RightFrontMotor; // 1 - base
    DcMotorEx LeftBackMotor; // 2 - base
    DcMotorEx RightBackMotor; // 3 - base
    DcMotorEx armMotor; // 0 - arm
    Servo claw; // 0 - arm

    @Override
    public void runOpMode()
    {

        LeftFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftFrontMotor");
        RightFrontMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightFrontMotor");
        LeftBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftBackMotor");
        RightBackMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightBackMotor");
        armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");

        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");

        // vv April Tags stuff DO NOT TOUCH vv
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        // ^^ April Tags stuff DO NOT TOUCH ^^

        /* Actually do something useful */

        /* User Snippet:
        if(override == true){
            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() $1$ (all * $POS$)));
            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() $2$ (all * $POS$)));
            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() $2$ (all * $POS$)));
            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() $1$ (all * $POS$)));
        } else {
            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() $1$ (LF * $POS$)));
            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() $2$ (RF * $POS$)));
            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() $2$ (LB * $POS$)));
            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() $1$ (RB * $POS$)));
        }
        LeftFrontMotor.setPower((int) (PowerFactor * $POWER1$));
        RightFrontMotor.setPower((int) (PowerFactor * $POWER2$));
        LeftBackMotor.setPower((int) (PowerFactor * $POWER2$));
        RightBackMotor.setPower((int) (PowerFactor * $POWER1$));
         */

        //TODO: Try this first
        if(override == true){
            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() + (all * 1)));
            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() + (all * 1)));
            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() + (all * 1)));
            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() + (all * 1)));
        } else {
            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() + (LF * 1)));
            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() + (RF * 1)));
            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() + (LB * 1)));
            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() + (RB * 1)));
        }
        LeftFrontMotor.setPower((int) (PowerFactor * 0.5));
        RightFrontMotor.setPower((int) (PowerFactor * 0.5));
        LeftBackMotor.setPower((int) (PowerFactor * 0.5));
        RightBackMotor.setPower((int) (PowerFactor * 0.5));
        
//        if(override == true){
//            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() + (all * 0.1)));
//            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() + (all * 0.1)));
//            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() + (all * 0.1)));
//            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() + (all * 0.1)));
//        } else {
//            LeftFrontMotor.setTargetPosition((int) (LeftFrontMotor.getCurrentPosition() + (LF * 0.1)));
//            RightFrontMotor.setTargetPosition((int) (RightFrontMotor.getCurrentPosition() + (RF * 0.1)));
//            LeftBackMotor.setTargetPosition((int) (LeftBackMotor.getCurrentPosition() + (LB * 0.1)));
//            RightBackMotor.setTargetPosition((int) (RightBackMotor.getCurrentPosition() + (RB * 0.1)));
//        }
//        LeftFrontMotor.setPower((int) (PowerFactor * 0.6));
//        RightFrontMotor.setPower((int) (PowerFactor * 0.1));
//        LeftBackMotor.setPower((int) (PowerFactor * 0.1));
//        RightBackMotor.setPower((int) (PowerFactor * 0.1));
//        armMotor.setPower(-.1 * PowerFactor);
        sleep(50);

        /*LeftFrontMotor.setPower(0.0);
        RightFrontMotor.setPower(0.0);
        LeftBackMotor.setPower(0.0);
        RightBackMotor.setPower(0.0);
        armMotor.setPower(-.12*PowerFactor);

        // close claw
        claw.setPosition(1.0);
        sleep(200);

        //Strafe right
        RightBackMotor.setPower(.5*PowerFactor);
        RightFrontMotor.setPower(-.5*PowerFactor);
        LeftFrontMotor.setPower(.5*PowerFactor);
        LeftBackMotor.setPower(-.5*PowerFactor);
        sleep(1550);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);

        sleep(1000);

        // move forward
        RightBackMotor.setPower(.5*PowerFactor);
        RightFrontMotor.setPower(.5*PowerFactor);
        LeftFrontMotor.setPower(.5*PowerFactor);
        LeftBackMotor.setPower(.5*PowerFactor);
        sleep(1900);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        sleep(100);

        // strafe left and score cone
        RightBackMotor.setPower(-.5*PowerFactor);
        RightFrontMotor.setPower(.5*PowerFactor);
        LeftFrontMotor.setPower(-.5*PowerFactor);
        LeftBackMotor.setPower(.5*PowerFactor);
        armMotor.setPower(-0.93*PowerFactor);
        sleep(675);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        sleep(1150);

        RightBackMotor.setPower(0.15*PowerFactor);
        RightFrontMotor.setPower(0.15*PowerFactor);
        LeftFrontMotor.setPower(0.15*PowerFactor);
        LeftBackMotor.setPower(0.15*PowerFactor);
        sleep(1050);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        claw.setPosition(0.0);
        armMotor.setPower(0.45*PowerFactor);
        RightBackMotor.setPower(-0.2*PowerFactor);
        RightFrontMotor.setPower(-0.2*PowerFactor);
        LeftFrontMotor.setPower(-0.2*PowerFactor);
        LeftBackMotor.setPower(-0.2*PowerFactor);
        sleep(400);

        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        sleep(2600);

        armMotor.setPower(-.1);

        //TODO: strafe to proper side
        if (tagOfInterest == null ||tagOfInterest.id == left) {
            RightBackMotor.setPower(-.5*PowerFactor);
            RightFrontMotor.setPower(.5*PowerFactor);
            LeftFrontMotor.setPower(-.5*PowerFactor);
            LeftBackMotor.setPower(.5*PowerFactor);
            sleep(1950);
            RightBackMotor.setPower(0);
            RightFrontMotor.setPower(0);
            LeftFrontMotor.setPower(0);
            LeftBackMotor.setPower(0);
        } else if (tagOfInterest == null ||tagOfInterest.id == right) {
            RightBackMotor.setPower(.5*PowerFactor);
            RightFrontMotor.setPower(-.5*PowerFactor);
            LeftFrontMotor.setPower(.5*PowerFactor);
            LeftBackMotor.setPower(-.5*PowerFactor);
            sleep(810);
            RightBackMotor.setPower(0);
            RightFrontMotor.setPower(0);
            LeftFrontMotor.setPower(0);
            LeftBackMotor.setPower(0);
        } else {
            RightBackMotor.setPower(-.5*PowerFactor);
            RightFrontMotor.setPower(.5*PowerFactor);
            LeftFrontMotor.setPower(-.5*PowerFactor);
            LeftBackMotor.setPower(.5*PowerFactor);
            sleep(810);
            RightBackMotor.setPower(0);
            RightFrontMotor.setPower(0);
            LeftFrontMotor.setPower(0);
            LeftBackMotor.setPower(0);
        }

        armMotor.setTargetPosition(0);*/

        sleep(999999);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }
}