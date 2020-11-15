package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Tuner;


@TeleOp(name = "AngularRomegaTeleop", group = "drive")

public class AngularRomegaTeleop extends OpMode {

    private Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"linCoeff", "angCoeff", "halfDistBetWheels", "singlestick", "velo PID", "invert back"};
    private double[] values = new double[] {     0.7  ,      1  ,          0.2         ,       1      ,      1    ,       -1      };

    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, false, false);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    public void loop(){
        tuner.tune();

        double linVelo = -gamepad1.left_stick_y;

        double turnStick;
        if(tuner.get("singlestick") > 0){
            turnStick = -gamepad1.left_stick_x;
        }else{
            turnStick = -gamepad1.right_stick_x;
        }

        double angVelo;
        if(tuner.get("invert back") > 0){
            angVelo  = -Math.atan2(turnStick, linVelo) / (0.5 * Math.PI);
        }else{
            angVelo  = -Math.atan2(turnStick, Math.abs(linVelo)) / (0.5 * Math.PI);
        }


        double leftVelo =  linVelo * tuner.get("linCoeff") + angVelo * tuner.get("halfDistBetWheels") * tuner.get("angCoeff");
        double rightVelo =  linVelo  * tuner.get("linCoeff") - angVelo * tuner.get("halfDistBetWheels") * tuner.get("angCoeff");

        double maxVelo = Math.max(leftVelo, rightVelo); //to keep the ratio between L and R velo
        if(maxVelo > 1){
            leftVelo /= maxVelo;
            rightVelo /= maxVelo;
        }

        if(tuner.get("velo PID") > 0){
            hardware.drivetrain.setVelocities(leftVelo, rightVelo);
        }else{
            hardware.drivetrain.setPowers(leftVelo, rightVelo);
        }

        telemetry.addData("linVelo", linVelo);
        telemetry.addData("angVelo", angVelo);
//        telemetry.addData("turnCenter", turnCenter);
        telemetry.addData("leftVelo target", leftVelo);
        telemetry.addData("rightVelo target", rightVelo);
        telemetry.update();
    }
}
