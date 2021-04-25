package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.ButtonPress;
import org.firstinspires.ftc.teamcode.lib.Tuner;
import org.firstinspires.ftc.teamcode.lib.Tuner.Var;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.Util.PID;

import static java.lang.Math.toRadians;

@TeleOp
public class TeleopFlippy2 extends OpMode {


    Tuner tuner;
    _Hardware hardware;

    ButtonPress startPress = new ButtonPress();
    ButtonPress yPress = new ButtonPress();

    double armLowSetPos, armTopSetPos = 0;

    @Override
    public void init() {
        tuner = new Tuner(
                gamepad1,
                telemetry,
                new Var("ostrichRest", 0.63),
                new Var("ostrichShoot", 0.7),
                new Var("ostrichIntake", 0.3),
                new Var("shooterPower", 0.99),
                new Var("forwardCoeff", 0.95),
                new Var("turnCoeff", -0.7),
                new Var("quickTurn", -0.7),
                new Var("shooterPower", 0.95),
                new Var("lowSetPosA", -0.315),
                new Var("topSetPosA", -1.14),
                new Var("lowSetPosB", 1.34),
                new Var("topSetPosB", 3.34),
                new Var("antibacklash", 0),
                new Var("lowP", 0.6),
                new Var("lowI", 0.14),
                new Var("lowD", 0.08),
                new Var("lowFx", 0.08),
                new Var("topP", 0.95),
                new Var("topI", 1.46),
                new Var("topD", 0.0),
                new Var("topFx", 0.11),
                new Var("antigravLow", 0.1),
                new Var("antigravTop", 0.05),
                new Var("lowMaxPower", 0.9)

            );

        hardware = new _Hardware(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        tuner.tune();

        hardware.flippy.K_antibacklash = tuner.get("antibacklash");
        hardware.flippy.K_antigravLow = tuner.get("antigravLow");
        hardware.flippy.K_antigravTop = tuner.get("antigravTop");
        hardware.flippy.K_lowP = tuner.get("lowP");
        hardware.flippy.K_lowI = tuner.get("lowI");
        hardware.flippy.K_lowD = tuner.get("lowD");
        hardware.flippy.K_lowFx = tuner.get("lowFx");
        hardware.flippy.K_topP = tuner.get("topP");
        hardware.flippy.K_topI = tuner.get("topI");
        hardware.flippy.K_topD = tuner.get("topD");
        hardware.flippy.K_topFx = tuner.get("topFx");
        hardware.flippy.K_armLowMaxPower = tuner.get("lowMaxPower");

        if(Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1){
            armLowSetPos += -gamepad2.left_stick_y * 0.01;
            armTopSetPos += -gamepad2.right_stick_y * 0.01;
        }
        if(gamepad1.a){
            armLowSetPos = tuner.get("lowSetPosA");
            armTopSetPos = tuner.get("topSetPosA");
            hardware.ostrichServo.setPosition(tuner.get("ostrichIntake"));
        }else if(gamepad1.b){
            armLowSetPos = tuner.get("lowSetPosB");
            armTopSetPos = tuner.get("topSetPosB");
            hardware.ostrichServo.setPosition(tuner.get("ostrichRest"));
        }else if(gamepad1.y){
            hardware.ostrichServo.setPosition(tuner.get("ostrichShoot"));
        }else{
            if(yPress.status(gamepad1.y) == ButtonPress.Status.COMPLETE){
                hardware.ostrichServo.setPosition(tuner.get("ostrichRest"));
            }
        }



        hardware.flippy.set(armLowSetPos, armTopSetPos);

        if(startPress.status(gamepad1.start) == ButtonPress.Status.COMPLETE){
            if(hardware.intake.getPower() > 0){
                hardware.intake.setPower(0);
            }else{
                hardware.intake.setPower(0.9);
            }
        }

        //SHOOTER
        double shooterPower = tuner.get("shooterPower");
        if(gamepad1.dpad_up){
            hardware.shooter.setPower(shooterPower);
        }else if(gamepad1.dpad_down){
            hardware.shooter.setPower(-shooterPower);
        }else if(gamepad1.dpad_left || gamepad1.dpad_right){
            hardware.shooter.setPower(0);
        }

        //DRIVETRAIN
        double forward = -gamepad1.left_stick_y * tuner.get("forwardCoeff");
        double turn = -gamepad1.right_stick_x * tuner.get("turnCoeff");

        boolean isQuickTurn = Math.abs(turn/forward) > tuner.get("quickTurn");

        if(!isQuickTurn && forward < 0){
            turn = -turn;
        }

        double[] powers = TeleopCheese2.cheesyDrive(forward, turn, isQuickTurn, true);
        hardware.drivetrain.setPowers(powers[0], powers[1]);


        telemetry.addData("armLow1Error", hardware.flippy.armLow1PID.error);
        telemetry.addData("armLow2Error", hardware.flippy.armLow2PID.error);
        telemetry.addData("armTopError", hardware.flippy.armTopPID.error);
        telemetry.addData("armLow1Angle", hardware.flippy.armLow1Angle);
        telemetry.addData("armLow2Angle", hardware.flippy.armLow2Angle);
        telemetry.addData("armTopAngle", hardware.flippy.armTopAngle);
        telemetry.addData("armHertz", hardware.flippy.hertz);
        telemetry.update();
    }

    @Override
    public void stop(){
        hardware.stop();
    }


}
