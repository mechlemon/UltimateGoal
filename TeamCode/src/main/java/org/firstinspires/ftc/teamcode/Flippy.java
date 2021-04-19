package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.ButtonPress;
import org.firstinspires.ftc.teamcode.lib.Tuner;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.Util.PID;

import static java.lang.Math.toRadians;

@TeleOp
public class Flippy extends OpMode {

    DcMotorEx leftDriveMotor;
    DcMotorEx rightDriveMotor;

    DcMotorEx shooterMotor;

    DcMotorEx armTopMotor;
    DcMotorEx armLow1Motor;
    DcMotorEx armLow2Motor;

    DcMotorEx intakeMotor;

    Servo ostrichServo;

    Tuner tuner;

    PID armLow1PID = new PID();
    PID armLow2PID = new PID();
    PID armTopPID = new PID();

    boolean enablePID = false;
    double lowSetPos, topSetPos;

    double armLowMaxPower = 0.7;
    double armTopMaxPower = 0.9;

    ButtonPress startPress = new ButtonPress();


    public void init() {
        tuner = new Tuner(
                new String[] {"forwardCoeff", "turnCoeff", "quickTurn", "lowOffset", "lowP", "lowFx", "topP", "topFx", "shootSetLow", "shootSetTop", "antigravLow", "antigravTop", "shooterPower", "ostrichLock", "ostrichShoot"},
                new double[] {0.9, -0.7, 3, 0.05, 0.5, 0.06, 0.5, 0.08, 1.2, -2.5, 0.16, -0.10, 1, 0.13, 0.3},
                gamepad1,
                telemetry
        );


        leftDriveMotor = hardwareMap.get(DcMotorEx.class, "2-0");
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor = hardwareMap.get(DcMotorEx.class, "2-1");



        shooterMotor = hardwareMap.get(DcMotorEx.class, "1-3");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        ostrichServo = hardwareMap.get(Servo.class, "1-0s");


        armLow1Motor = hardwareMap.get(DcMotorEx.class, "1-0");
        armLow1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLow2Motor = hardwareMap.get(DcMotorEx.class, "1-1");
        armLow2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLow2Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        armTopMotor = hardwareMap.get(DcMotorEx.class, "1-2");
        armTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "2-3");


        resetStartTime();
    }

    double shootLastTick = 0;
    double lastTime = 0;

    double intakePower = 0;

    double servoPos = 0.1;

    public void loop() {
        double dt = getRuntime() - lastTime;
        lastTime = getRuntime();

        tuner.tune();

        //DRIVETRAIN
        double forward = -gamepad1.left_stick_y * tuner.get("forwardCoeff");
        double turn = -gamepad1.right_stick_x * tuner.get("turnCoeff");

        boolean isQuickTurn = Math.abs(turn/forward) > tuner.get("quickTurn");

        if(!isQuickTurn && forward < 0){
            turn = -turn;
        }

        double[] powers = CheeseTeleop2.cheesyDrive(forward, turn, isQuickTurn, true);
        leftDriveMotor.setPower(powers[0]);
        rightDriveMotor.setPower(powers[1]);


        //SHOOTER
        double shooterPower = tuner.get("shooterPower");

        if(gamepad1.dpad_up){
            shooterMotor.setPower(shooterPower);
        }else if(gamepad1.dpad_down){
            shooterMotor.setPower(-shooterPower);
        }else if(gamepad1.dpad_left || gamepad1.dpad_right){
            shooterMotor.setPower(0);
        }

        double shootRPM = (1/28.0) * (60) * (shooterMotor.getCurrentPosition() - shootLastTick) / dt;
        shootLastTick = shooterMotor.getCurrentPosition();

        //SERVO
        if(gamepad1.y){
            servoPos = (tuner.get("ostrichShoot"));
        }else{
            servoPos = (tuner.get("ostrichLock"));
        }
        ostrichServo.setPosition(servoPos);

        //ARM
        if(gamepad1.b){
            lowSetPos = tuner.get("shootSetLow");
            topSetPos = tuner.get("shootSetTop");
            enablePID = true;
        }else if(gamepad1.a){
            lowSetPos = toRadians(3);
            topSetPos = -0.1;
            enablePID = true;
        }else if(Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1){
            enablePID = false;
        }

        //INTAKE
        if(startPress.status(gamepad1.start) == ButtonPress.Status.COMPLETE){
            if(intakePower > 0){
                intakePower = 0;
            }else{
                intakePower = 1;
            }
        }
        intakeMotor.setPower(intakePower);


            double armLow1Angle = (armLow1Motor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (36.0/16.0));
        double armLow2Angle = (armLow1Motor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (36.0/16.0));
        double antigravLow = tuner.get("antigravLow") * Math.cos(armLow1Angle);

        double armTopAngle = (armTopMotor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (34.0/16.0)) + armLow1Angle;
        double antigravTop = tuner.get("antigravTop") * Math.cos(armTopAngle);

        armLow1PID.setkP(tuner.get("lowP"));
        armLow1PID.setkFx(tuner.get("lowFx"), toRadians(3));
        armLow2PID.copyConstants(armLow1PID);
        armTopPID.setkP(tuner.get("topP"));
        armTopPID.setkFx(tuner.get("topFx"), toRadians(3));

        if(enablePID){
            armLow1PID.loop(armLow1Angle, calcClosestAngle360(armLow1Angle, lowSetPos), dt);
            armLow2PID.loop(armLow2Angle, calcClosestAngle360(armLow2Angle, lowSetPos + tuner.get("lowOffset")), dt);
            armTopPID.loop(armTopAngle, topSetPos, dt);


            armLow1Motor.setPower(Range.clip(armLow1PID.getPower() + antigravLow, -armLowMaxPower, armLowMaxPower));
            armLow2Motor.setPower(Range.clip(armLow2PID.getPower() + antigravLow, -armLowMaxPower, armLowMaxPower));
            armTopMotor.setPower(Range.clip(armTopPID.getPower() + antigravTop, -armTopMaxPower, armTopMaxPower));

        }else{
            armLow1Motor.setPower(-gamepad2.left_stick_y*armLowMaxPower + antigravLow);
            armLow2Motor.setPower(-gamepad2.left_stick_y*armLowMaxPower + antigravLow);
            armTopMotor.setPower(-gamepad2.right_stick_y*armTopMaxPower + antigravTop);
        }



        telemetry.addData("shootRPM", shootRPM);
        telemetry.addData("armLow1Error", armLow1PID.error);
        telemetry.addData("armLow2Error", armLow2PID.error);
        telemetry.addData("armTopError", armTopPID.error);
        telemetry.addData("armLow1Angle", armLow1Angle);
        telemetry.addData("armLow2Angle", armLow2Angle);
        telemetry.addData("armTopAngle", armTopAngle);
        telemetry.update();
    }

    private double calcClosestAngle360(double currentAngle, double targetAngle){
        double diffNormalized = Util.normalizeAngle(currentAngle - targetAngle, Math.PI); //angle error from (-PI, PI)
        return currentAngle - diffNormalized;
    }


}
