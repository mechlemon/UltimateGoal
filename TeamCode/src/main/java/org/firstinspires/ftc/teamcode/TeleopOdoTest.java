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


@TeleOp

public class TeleopOdoTest extends OpMode {

    private _Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"linCoeff", "angCoeff"};
    private double[] values = new double[] {    0.7  ,    0.3    };


    public void init(){
        hardware = new _Hardware(hardwareMap, telemetry, true);
        tuner = new Tuner(titles, values, gamepad1, telemetry);


    }

    public void loop(){
        tuner.tune();

        double forward = -gamepad1.left_stick_y * tuner.get("linCoeff");
        double turn = -gamepad1.right_stick_x * tuner.get("angCoeff");

        hardware.drivetrain.plusMinusDrive(forward, turn);

        if(gamepad1.x){
            hardware.drivetrain.reset();
        }

        hardware.drivetrain.updateOdo();


        telemetry.addData(
                "odoPose1",
                hardware.drivetrain.robotPose1.toString()
        );
        telemetry.addData(
                "odoPose2",
                hardware.drivetrain.robotPose2.toString()
        );
        telemetry.addData(
                "LR Dists",
                Util.roundHundreths(hardware.drivetrain.getLeftDistance()) + ", " +
                        Util.roundHundreths(hardware.drivetrain.getRightDistance()) + ", " +
                        Util.roundHundreths(hardware.drivetrain.getHorizDistance())
        );

        telemetry.update();
    }
}
