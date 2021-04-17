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

    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;

    DcMotorEx armTopMotor;

    Servo ostrichServo;

    Tuner tuner;

    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "1-2");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "1-3");

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        armTopMotor = hardwareMap.get(DcMotorEx.class, "1-1");
        armTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ostrichServo = hardwareMap.get(Servo.class, "1-0s");

        tuner = new Tuner(
                new String[] {"antigrav", "leftShooterPower", "rightShooterPower", "ostrichLock", "ostrichShoot"},
                new double[] {0.14, 1, 1, 0.38, 0.72},
                gamepad1,
                telemetry
        );
    }

    double leftLastTick = 0;
    double rightLastTick = 0;
    double lastTime = 0;

    public void loop() {
        tuner.tune();

        double leftShooterPower = tuner.get("leftShooterPower");
        double rightShooterPower = tuner.get("rightShooterPower");

        if(gamepad1.dpad_up){
            leftShooterMotor.setPower(leftShooterPower);
            rightShooterMotor.setPower(rightShooterPower);
        }else if(gamepad1.dpad_down){
            leftShooterMotor.setPower(-leftShooterPower);
            rightShooterMotor.setPower(-rightShooterPower);
        }else if(gamepad1.dpad_left || gamepad1.dpad_right){
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

        if(gamepad1.y){
            ostrichServo.setPosition(tuner.get("ostrichShoot"));
        }else{
            ostrichServo.setPosition(tuner.get("ostrichLock"));
        }

        double armTopAngle = (armTopMotor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (34.0/16.0));
        double antigrav = tuner.get("antigrav") * Math.sin(armTopAngle);
        armTopMotor.setPower(-gamepad1.right_stick_y + antigrav);

        double dt = getRuntime() - lastTime;
        lastTime = getRuntime();
        double leftRPM = (1/28.0) * (60) * (leftShooterMotor.getCurrentPosition() - leftLastTick) / dt;
        double rightRPM = (1/28.0) * (60) * (rightShooterMotor.getCurrentPosition() - rightLastTick) / dt;
        leftLastTick = leftShooterMotor.getCurrentPosition();
        rightLastTick = rightShooterMotor.getCurrentPosition();

        telemetry.addData("leftRPM", leftRPM);
        telemetry.addData("rightRPM", rightRPM);
        telemetry.addData("ostrichPos", ostrichServo.getPosition());
        telemetry.addData("armPower", armTopMotor.getPower());
        telemetry.addData("armTopAngle", armTopAngle);
        telemetry.update();
    }


}
