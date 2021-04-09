package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.Tuner;


@TeleOp(name = "RomegaTeleop", group = "drive")git

public class RomegaTeleop extends OpMode {

    private _Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"linCoeff", "angCoeff"};
    private double[] values = new double[] {     0.7  ,      1    };

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

        hardware.drivetrain.setVelocities(leftVelo, rightVelo);


        DcMotorEx leftMotor = hardware.drivetrain.leftMotor;
        DcMotorEx rightMotor = hardware.drivetrain.rightMotor;
        double maxTicksPerSecond = leftMotor.getMotorType().getAchieveableMaxTicksPerSecond();

        telemetry.addData("odoPose", hardware.drivetrain.robotPose.toString());
        telemetry.addData("tarVelos", leftVelo + ", " + rightVelo);
        telemetry.addData(
                "curVelos",
                leftMotor.getVelocity() / maxTicksPerSecond + ", " + rightMotor.getVelocity() / maxTicksPerSecond);
        telemetry.update();
    }
}
