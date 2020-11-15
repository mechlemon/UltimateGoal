package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Tuner;


@TeleOp(name = "AdditionTeleop", group = "drive")

public class AdditionTeleop extends OpMode {

    private Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"turnCoeff", "singlestick"};
    private double[] values = new double[] {     0.7   ,       -1     };

    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, false, false);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    public void loop(){
        tuner.tune();

        double forward = -gamepad1.left_stick_y;

        double turn;
        if(tuner.get("singlestick") > 0){
            turn = -gamepad1.left_stick_x * tuner.get("turnCoeff");
        }else{
            turn = -gamepad1.right_stick_x * tuner.get("turnCoeff");
        }

        double leftPower = forward - turn;
        double rightPower = forward + turn;

        double maxVelo = Math.max(leftPower, rightPower); //to keep the ratio between L and R power
        if(maxVelo > 1){
            leftPower /= maxVelo;
            rightPower /= maxVelo;
        }

        hardware.drivetrain.setPowers(leftPower, rightPower);

        telemetry.addData("forward", forward);
        telemetry.addData("turn", turn);
        telemetry.addData("left inches", hardware.drivetrain.getLeftDistance());
        telemetry.addData("right inches", hardware.drivetrain.getRightDistance());
        telemetry.update();
    }
}
