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

import org.firstinspires.ftc.teamcode.lib.Util;
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
public class AutoBlue extends LinearOpMode
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

            if (pipeline.CrAvg > 160) {
                numRings = 4; //144
            } else if (pipeline.CrAvg > 135) {
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

        Util.PID turnPID = new Util.PID();
        turnPID.setkP(0.7);
        turnPID.setkI(0.1, 0.4, 0.5);

        while(opModeIsActive()){

            if(status == "start"){
                hardware.drivetrain.setPowers(0.5, 0.5);
                if(hardware.drivetrain.getAvgForwardDist() > 30){
                    status = "push";
                }
            }

            if(status == "push" && numRings == 0) {
                hardware.drivetrain.setPowers(0.2, 0.6);
                if (hardware.drivetrain.getAvgForwardDist() > 60) {
                    status = "back";
                }
            }

            if(status == "back" && numRings == 0) {
                hardware.drivetrain.setPowers(-0.2, -0.6);
                if (hardware.drivetrain.getAvgForwardDist() < 50) {
                    hardware.stop();
                    status = "spin wheels";
                }
            }

            if(status == "push" && numRings == 1) {
                hardware.drivetrain.setPowers(0.6, 0.4);
                if (hardware.drivetrain.getAvgForwardDist() > 80) {
                    status = "back";
                }
            }

            if(status == "back" && numRings == 1) {
                hardware.drivetrain.setPowers(-0.6, -0.4);
                if (hardware.drivetrain.getAvgForwardDist() < 50) {
                    hardware.stop();
                    status = "spin wheels";
                }
            }

            if(status == "push" && numRings == 4) {
                hardware.drivetrain.setPowers(0.5, 0.6);
                if (hardware.drivetrain.getAvgForwardDist() > 100) {
                    status = "back";
                }
            }

            if(status == "back" && numRings == 4) {
                hardware.drivetrain.setPowers(-0.5, -0.6);
                if (hardware.drivetrain.getAvgForwardDist() < 50) {
                    hardware.stop();
                    status = "spin wheels";
                }
            }

            if(status == "spin wheels"){
                hardware.setShooterRPM(42);
                resetStartTime();
                status = "wait for speed";
                resetStartTime();
            }

            if(status == "wait for speed"){
                double turnPower = turnPID.loop(hardware.imu.getHeading(), -0.45, 0.02);
                hardware.drivetrain.setPowers(-turnPower, turnPower);
                if(getRuntime() > 5){
                    status = "shoot1";
                    resetStartTime();
                }
            }

            if(status == "shoot1"){
                hardware.shoot(1, getRuntime());
                if(getRuntime() > 1){
                    status = "align2";
                }
            }

            if(status == "align2"){
                double turnPower = turnPID.loop(hardware.imu.getHeading(), -0.50, 0.02);
                hardware.drivetrain.setPowers(-turnPower, turnPower);
                if(getRuntime() > 3){
                    status = "shoot2";
                    resetStartTime();
                }
            }

            if(status == "shoot2"){
                hardware.shoot(1, getRuntime());
                if(getRuntime() > 1){
                    status = "align3";
                }
            }

            if(status == "align3"){
                double turnPower = turnPID.loop(hardware.imu.getHeading(), -0.55, 0.02);
                hardware.drivetrain.setPowers(-turnPower, turnPower);
                if(getRuntime() > 3){
                    status = "shoot3";
                    resetStartTime();
                }
            }

            if(status == "shoot3"){
                hardware.shoot(1, getRuntime());
                if(getRuntime() > 1){
                    status = "park";
                }
            }

            if(status == "park"){
                hardware.drivetrain.setPowers(0.5, 0.5);
                if (hardware.drivetrain.getAvgForwardDist() > 70) {
                    status = "stop";
                }
            }

            if(status == "stop"){
                hardware.stop();
            }





            telemetry.addData("dist", hardware.drivetrain.getAvgForwardDist());
            telemetry.addData("status", status);

            telemetry.update();
        }

    }


    public static class RingsPipeline extends OpenCvPipeline {

        static final Scalar BLUE = new Scalar(0, 0, 255);

        Point region1_pointA = new Point(
                50,
                70);
        Point region1_pointB = new Point(
                90,
                100);

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