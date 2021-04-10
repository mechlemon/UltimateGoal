package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.Tuner;

@TeleOp(name = "Shooter", group = "test")

public class Shooter extends OpMode {

    DcMotorEx motor;

    Tuner tuner;

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "1-2");
        tuner = new Tuner(
                new String[] {"speed"},
                new double[] { 0.35  },
                gamepad1,
                telemetry
        );
    }

    public void loop() {
        tuner.tune();
        double target = tuner.get("speed");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(target);

        double velo = motor.getVelocity() / motor.getMotorType().getAchieveableMaxTicksPerSecond();

        telemetry.addData( "velo", velo);
        telemetry.update();

    }
}
