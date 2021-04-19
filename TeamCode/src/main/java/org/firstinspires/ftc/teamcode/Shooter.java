package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Tuner;

@TeleOp
public class Shooter extends OpMode {

    DcMotorEx shooterMotor;

    DcMotorEx armTopMotor;
    DcMotorEx armLow1Motor;
    DcMotorEx armLow2Motor;

    Servo ostrichServo;

    Tuner tuner;

    public void init() {

        armLow1Motor = hardwareMap.get(DcMotorEx.class, "1-0");
        armLow1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLow2Motor = hardwareMap.get(DcMotorEx.class, "1-1");
        armLow2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLow2Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        armTopMotor = hardwareMap.get(DcMotorEx.class, "1-2");
        armTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "1-3");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        ostrichServo = hardwareMap.get(Servo.class, "1-0s");

        tuner = new Tuner(
                new String[] {"antigravLow", "antigravTop", "shooterPower", "ostrichLock", "ostrichShoot"},
                new double[] {0.14, 0.14, 1, 0.38, 0.72},
                gamepad1,
                telemetry
        );
    }

    double shootLastTick = 0;
    double lastTime = 0;

    public void loop() {
        tuner.tune();

        double shooterPower = tuner.get("shooterPower");

        if(gamepad1.dpad_up){
            shooterMotor.setPower(shooterPower);
        }else if(gamepad1.dpad_down){
            shooterMotor.setPower(-shooterPower);
        }else if(gamepad1.dpad_left || gamepad1.dpad_right){
            shooterMotor.setPower(0);
        }

        if(gamepad1.y){
            ostrichServo.setPosition(tuner.get("ostrichShoot"));
        }else{
            ostrichServo.setPosition(tuner.get("ostrichLock"));
        }

        double armLowAngle = (armLow1Motor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (36.0/16.0));
        double antigravLow = tuner.get("antigravLow") * Math.sin(armLowAngle);
        armLow1Motor.setPower(-gamepad1.left_stick_y + antigravLow);
        armLow2Motor.setPower(-gamepad1.left_stick_y + antigravLow);

        double armTopAngle = (armTopMotor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (34.0/16.0)) + armLowAngle;
        double antigravTop = tuner.get("antigravTop") * Math.sin(armTopAngle);
        armTopMotor.setPower(-gamepad1.right_stick_y + antigravTop);

        double dt = getRuntime() - lastTime;
        lastTime = getRuntime();
        double shootRPM = (1/28.0) * (60) * (shooterMotor.getCurrentPosition() - shootLastTick) / dt;
        shootLastTick = shooterMotor.getCurrentPosition();

        telemetry.addData("shootRPM", shootRPM);
        telemetry.addData("ostrichPos", ostrichServo.getPosition());
        telemetry.addData("armPower", armTopMotor.getPower());
        telemetry.addData("armLowAngle", armLowAngle);
        telemetry.addData("armTopAngle", armTopAngle);
        telemetry.update();
    }


}
