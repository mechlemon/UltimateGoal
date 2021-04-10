package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Tuner;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


@TeleOp(name = "RomegaTeleop", group = "drive")

public class RomegaTeleop extends OpMode {

    private _Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"linCoeff", "angCoeff"};
    private double[] values = new double[] {    -0.7  ,    0.5    };

    OpenCvCamera phoneCam;

    public void init(){
        hardware = new _Hardware(hardwareMap, telemetry);
        tuner = new Tuner(titles, values, gamepad1, telemetry);


    }

    public void loop(){
        tuner.tune();

        double linVelo = -gamepad1.left_stick_y * tuner.get("linCoeff");

        double angVelo = -gamepad1.right_stick_x * tuner.get("angCoeff");

        double r = hardware.drivetrain.TRACK_WIDTH * 0.5;

        double leftVelo =  linVelo + angVelo * r;
        double rightVelo =  linVelo - angVelo * r;

        double maxVelo = Math.max(leftVelo, rightVelo); //to keep the ratio between L and R velo
        if(maxVelo > 1){
            leftVelo /= maxVelo;
            rightVelo /= maxVelo;
        }

        hardware.drivetrain.setPowers(leftVelo, rightVelo);

        if(gamepad1.x){
            hardware.drivetrain.resetEncoders();
            hardware.drivetrain.robotPose = new Pose2D();
        }


        telemetry.addData(
                "odoPose",
                hardware.drivetrain.robotPose.toString());
        telemetry.addData(
                "LR Dists",
                Util.roundHundreths(hardware.drivetrain.getLeftDistance()) + ", " + Util.roundHundreths(hardware.drivetrain.getRightDistance()));
        telemetry.addData(
                "H Dist",
                hardware.drivetrain.getHorizDistance());
        telemetry.update();
    }
}
