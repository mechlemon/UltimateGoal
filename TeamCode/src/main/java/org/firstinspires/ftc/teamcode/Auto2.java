package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2020 OpenFTC Team
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



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@TeleOp
public class Auto2 extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    RingsPipeline pipeline;

    _Hardware hardware;

    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingsPipeline();
        phoneCam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        int numRings = 0;
        while(!isStarted()){

            if (pipeline.CrAvg > 153) {
                numRings = 4;
            } else if (pipeline.CrAvg > 133) {
                numRings = 1;
            }

            telemetry.addData("Cr", pipeline.CrAvg);
            telemetry.addData("numRings", numRings);
            telemetry.update();

            sleep(50);
        }

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        hardware = new _Hardware(hardwareMap, telemetry);

        String status = "start";
        hardware.drivetrain.resetEncoders();

        while(opModeIsActive()){
            if(hardware.drivetrain.getAvgForwardDist() < 30){
                hardware.drivetrain.setPowers(0.5, 0.5);
                status = "initForward";
            }else{
                if (numRings == 0) {
                    if (hardware.drivetrain.getAvgForwardDist() < 60){
                        hardware.drivetrain.setPowers(0.2, 0.6);
                        status = "turn left for 0";
                    }else{
                        hardware.stop();
                    }
                } else if (numRings == 1) {
                    if (hardware.drivetrain.getAvgForwardDist() < 80){
                        hardware.drivetrain.setPowers(0.6, 0.4);
                        status = "turn right for 1";
                    }else{
                        hardware.stop();
                    }
                }else if (numRings == 4) {
                    if (hardware.drivetrain.getAvgForwardDist() < 100){
                        hardware.drivetrain.setPowers(0.5, 0.6);
                        status = "turn left for 4";
                    }else{
                        hardware.stop();
                    }
                }
            }



            telemetry.addData("dist", hardware.drivetrain.getAvgForwardDist());
            telemetry.addData("status", status);

            telemetry.update();
        }

    }


    public static class RingsPipeline extends OpenCvPipeline {

        static final Scalar BLUE = new Scalar(0, 0, 255);

        Point region1_pointA = new Point(
                120,
                50);
        Point region1_pointB = new Point(
                150,
                90);

        Mat region1;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();

        volatile int CrAvg;


        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }


        @Override
        public void init(Mat firstFrame) {
            inputToCr(firstFrame);

            region1 = Cr.submat(new Rect(region1_pointA, region1_pointB));
        }


        @Override
        public Mat processFrame(Mat input) {
            inputToCr(input);

            CrAvg = (int) Core.mean(region1).val[0];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            return input;
        }
    }
}