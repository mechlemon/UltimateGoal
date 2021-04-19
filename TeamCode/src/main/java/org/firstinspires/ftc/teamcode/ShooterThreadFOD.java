package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ButtonPress;
import org.firstinspires.ftc.teamcode.lib.Tuner;

@TeleOp

public class ShooterThreadFOD extends OpMode {
    ButtonPress ypress = new ButtonPress();
    ButtonPress dpadUpPress = new ButtonPress();

    Tuner tuner;

    _Hardware hardware;

    double spatulaPos = 0.8;
    boolean shooterOn = false;


    @Override
    public void init() {

        hardware = new _Hardware(hardwareMap, telemetry);

        tuner = new Tuner(
                new String[] {"targetSpeed", "FOD", "turnCoeff", "driveCoeff"},
                new double[] { 0.35        ,   0.1,      0.5,          0.8   },
                gamepad1,
                telemetry
        );
        resetStartTime();
    }



    @Override
    public void loop() {
        tuner.tune();

        if(gamepad1.dpad_up){
            shooterOn = true;
        }else if(gamepad1.dpad_down){
            hardware.stop();
            shooterOn = false;
        }



        if(ypress.status(gamepad1.y) == ButtonPress.Status.COMPLETE){
            resetStartTime();
        }

        if(shooterOn){
            hardware.setShooterRPM(tuner.get("targetSpeed") * 6000);
            hardware.shoot(3, getRuntime());
        }

        if(gamepad1.a && spatulaPos < 1){
            spatulaPos += 0.05;
        }else if(gamepad1.b && spatulaPos > 0){
            spatulaPos -= 0.05;
        }

//        hardware.spatula.setPosition(spatulaPos);



        if(tuner.get("FOD") > 0){
            hardware.drivetrain.fieldOriented(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        }else{
            hardware.drivetrain.drive(-gamepad1.left_stick_y * tuner.get("driveCoeff"), -gamepad1.right_stick_x * tuner.get("turnCoeff"));
        }

//        telemetry.addData("yeetup", hardware.yeetup.getPosition());
//        telemetry.addData("spatula", hardware.spatula.getPosition());
        telemetry.addData( "rpm", hardware.currentRPM);
        telemetry.update();
    }

    @Override
    public void stop(){
        hardware.stop();
    }



}
