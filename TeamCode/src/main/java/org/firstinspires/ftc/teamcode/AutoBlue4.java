package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AngleTracker;
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


@TeleOp
public class AutoBlue4 extends LinearOpMode {

    _Hardware hardware;

    double lowSetPosA =  -0.315;
    double topSetPosA = -1.14;
    double lowSetPosB = 1.34;
    double topSetPosB = 3.34;

    AngleTracker contAngle = new AngleTracker(-Math.PI, Math.PI);


    @Override
    public void runOpMode() {

        hardware = new _Hardware(hardwareMap, telemetry, false);

        waitForStart();

        hardware.drivetrain.reset();

        hardware.ostrichRest();

        Util.PID turnPID = new Util.PID();
        turnPID.setkP(0.7);
        turnPID.setkI(0.1, 0.4, 0.5);
        turnPID.setkFx(0.2, 0.05);

        String status = "start";

        while(opModeIsActive()){

            contAngle.update(hardware.imu.getHeading());

            if(status == "start"){
                hardware.drivetrain.setPowers(0.5, 0.5);
                hardware.flippy.set(lowSetPosA, topSetPosA);
                if(hardware.drivetrain.getAvgForwardDist() > 30){
                    status = "push";
                }
            }

            if(status == "push") {
                hardware.drivetrain.setPowers(0.6, 0.5);
                hardware.intake.setPower(1);
                if (hardware.drivetrain.getAvgForwardDist() > 100) {
                    status = "back";
                    hardware.intake.setPower(0);

                }
            }

            if(status == "back") {
                hardware.drivetrain.setPowers(-0.6, -0.5);
                hardware.flippy.set(lowSetPosB, topSetPosB);
                if (hardware.drivetrain.getAvgForwardDist() < 55) {
                    hardware.stop();
                    status = "spin wheels";
                }
            }

            if(status == "spin wheels"){
                hardware.shooter.setPower(-1);
                resetStartTime();
                status = "wait for speed";
                resetStartTime();
            }

            if(status == "wait for speed"){
                double turnPower = turnPID.loop(contAngle.getContinuousAngle(), 3.14, 0.02);
                hardware.drivetrain.setPowers(turnPower, -turnPower);
                hardware.flippy.set(lowSetPosB, topSetPosB);
                if(getRuntime() > 5){
                    status = "shoot1";
                    hardware.drivetrain.setPowers(0, 0);
                    resetStartTime();
                }
            }

            if(status == "shoot1"){
                hardware.ostrichShoot();
                if(getRuntime() > 1){
                    status = "align2";
                    hardware.ostrichRest();
                    resetStartTime();
                }
            }

            if(status == "align2"){
//                double turnPower = turnPID.loop(hardware.imu.getHeading(), -0.50 + Math.PI, 0.02);
//                hardware.drivetrain.setPowers(-turnPower, turnPower);
                if(getRuntime() > 2){
                    status = "shoot2";
                    resetStartTime();
                }
            }

            if(status == "shoot2"){
                hardware.ostrichShoot();
                if(getRuntime() > 1){
                    hardware.ostrichRest();
                    status = "align3";
                    resetStartTime();
                }
            }

            if(status == "align3"){
//                double turnPower = turnPID.loop(hardware.imu.getHeading(), -0.55 + Math.PI, 0.02);
//                hardware.drivetrain.setPowers(-turnPower, turnPower);
                if(getRuntime() > 2){
                    status = "shoot3";
                    resetStartTime();
                }
            }

            if(status == "shoot3"){
                hardware.ostrichShoot();
                if(getRuntime() > 1){
                    status = "turnback";
                    hardware.ostrichIntake();
                    hardware.flippy.set(lowSetPosA, topSetPosA);
                    hardware.intake.setPower(1);
                    resetStartTime();
                }
            }

            if(status == "turnback"){
                double turnPower = turnPID.loop(contAngle.getContinuousAngle(), 0.65, 0.02);
                hardware.drivetrain.setPowers(turnPower, -turnPower);
                hardware.flippy.set(lowSetPosA, topSetPosA);


                if (getRuntime() > 3) {
                    status = "stop";
                }
            }

            if(status == "gointake"){
                hardware.drivetrain.setPowers(-0.5, -0.5);

                if (hardware.drivetrain.getAvgForwardDist() < 30) {
                    status = "stop";
                }
            }

            if(status == "stop"){
                hardware.stop();
            }






            telemetry.addData("dist", hardware.drivetrain.getAvgForwardDist());
            telemetry.addData("angle", contAngle.getContinuousAngle());

            telemetry.addData("status", status);

            telemetry.update();
        }

    }


}