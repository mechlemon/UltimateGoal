package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class Auto extends LinearOpMode
{

    OpenCvInternalCamera phoneCam;
    RingsPipeline pipeline;


    @Override
    public void runOpMode() {


        _Hardware hardware;

        int numRings = 0;

        hardware = new _Hardware(hardwareMap, telemetry);


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
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("avg", pipeline.CrAvg);
            telemetry.update();

        }




    }


//    public void loodp(){
//
//        if (pipeline.CrAvg > 140) {
//            numRings = 4;
//        } else if (pipeline.CrAvg > 120) {
//            numRings = 1;
//        }
//
//        telemetry.addData("yellowAvg", pipeline.CrAvg);
//        telemetry.addData("numRings", numRings);
//        telemetry.update();
//
//        phoneCam.stopStreaming();
//        phoneCam.closeCameraDevice();
//
//
//        if(hardware.drivetrain.getAvgForwardDist() < 30){
//            hardware.drivetrain.setPowers(0.5, 0.5);
//        }
//
//        if (numRings == 0) {
//            if (hardware.drivetrain.getAvgForwardDist() < 60){
//                hardware.drivetrain.setPowers(0.5, 0.6);
//            }else{
//                hardware.stop();
//            }
//        } else if (numRings == 1) {
//            if (hardware.drivetrain.getAvgForwardDist() < 70){
//                hardware.drivetrain.setPowers(0.55, 0.5);
//            }else{
//                hardware.stop();
//            }
//        }else if (numRings == 4) {
//            if (hardware.drivetrain.getAvgForwardDist() < 80){
//                hardware.drivetrain.setPowers(0.5, 0.51);
//            }else{
//                hardware.stop();
//            }
//        }
//    }


    public static class RingsPipeline extends OpenCvPipeline {



        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);


        Point region1_pointA = new Point(
                120,
                50);
        Point region1_pointB = new Point(
                150,
                90);

        /*
         * Working variables
         */
        Mat region1;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();

        volatile int CrAvg;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cr channel to the 'Cr' variable
         */
        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }


        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCr(firstFrame);


            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1 = Cr.submat(new Rect(region1_pointA, region1_pointB));
        }


        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */


            /*
             * Get the Cr channel of the input frame after conversion to YCrCb
             */
            inputToCr(input);


            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            CrAvg = (int) Core.mean(region1).val[0];


            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines


            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }
    }



}