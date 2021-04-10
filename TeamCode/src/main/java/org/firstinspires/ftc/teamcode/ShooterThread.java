package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ButtonPress;
import org.firstinspires.ftc.teamcode.lib.Tuner;

@TeleOp(name = "ShooterThread", group = "test")

public class ShooterThread extends OpMode {

    MotorThread motorThread;
    double speed = 0;

    Servo yeetup;
    boolean yeeting = false;

    Servo spatula;

    ButtonPress ypress = new ButtonPress();

    Tuner tuner;

    _Hardware hardware;

    @Override
    public void init() {

        hardware = new _Hardware(hardwareMap, telemetry);

        motorThread = new MotorThread("1-3");
        yeetup = hardwareMap.servo.get("yeetup");
        spatula = hardwareMap.servo.get("spatula");

        tuner = new Tuner(
                new String[] {"speed", "delay", "backpos", "frontpos"},
                new double[] { 0.35  ,    0.3  ,     0.79  ,   0.69   },
                gamepad1,
                telemetry
        );
        resetStartTime();
    }

    double spatulaPos = 0;

    @Override
    public void loop() {
        tuner.tune();
        speed = tuner.get("speed");
        double delay = tuner.get("delay");

        if(ypress.status(gamepad1.y) == ButtonPress.Status.COMPLETE){
            yeeting = true;
            resetStartTime();
        }

        if(getRuntime() < 3*delay && yeeting){

            if(getRuntime() % delay < 0.5*delay){
                yeetup.setPosition(tuner.get("frontpos"));
            }else{
                yeetup.setPosition(tuner.get("backpos"));
            }
        }

        double forward = -gamepad1.left_stick_y*0.8;
        double turn = -gamepad1.right_stick_x * 0.5;

        if(gamepad1.a){
            spatulaPos += 0.05;
        }else if(gamepad1.b){
            spatulaPos -= 0.05;
        }

        spatula.setPosition(spatulaPos);

        hardware.drivetrain.setPowers(forward + turn, forward - turn);

        telemetry.addData("yeetup", yeetup.getPosition());
        telemetry.addData("spatula", spatula.getPosition());
        telemetry.addData( "velo", motorThread.motor.getVelocity() / motorThread.motor.getMotorType().getAchieveableMaxTicksPerSecond());
        telemetry.update();
    }

    @Override
    public void stop(){
        motorThread.stop();
    }

    class MotorThread implements Runnable {
        DcMotorEx motor;
        boolean stopped = false;

        MotorThread(String motorName) {
            motor = hardwareMap.get(DcMotorEx.class, motorName);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            new Thread(this).start();
        }

        public void run(){
            while(!stopped){
                motor.setPower(speed);
            }
        }

        void stop(){
            stopped = true;
            motor.setPower(0);
        }
    }

}
