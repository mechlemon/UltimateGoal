package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Tuner;

@TeleOp(name = "CheeseTeleop", group = "drive")
public class CheeseTeleop extends OpMode {


    private Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"lowScalar", "turnCoeff", "lowTurnCoeff","quickturn cutoff", "singlestick", "invert back"};
    private double[] values = new double[] {    0.5    ,     0.58   ,        0.2    ,        10        ,       1      ,       1      };

    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, false, false);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    public void loop(){
        tuner.tune();

        double forward = -gamepad1.left_stick_y;

        double turn;
        if(tuner.get("singlestick") > 0){
            if(gamepad1.left_bumper){
                turn = -gamepad1.left_stick_x * tuner.get("lowTurnCoeff");
            }else{
                turn = -gamepad1.left_stick_x * tuner.get("turnCoeff");
            }
        }else{
            if(gamepad1.left_bumper){
                turn = -gamepad1.right_stick_x * tuner.get("lowTurnCoeff");
            }else{
                turn = -gamepad1.right_stick_x * tuner.get("turnCoeff");
            }
        }

        boolean isQuickTurn = Math.abs(turn/forward) > tuner.get("quickturn cutoff");


        if(gamepad1.right_bumper){
            forward *= tuner.get("lowScalar");
        }


        if(!isQuickTurn && tuner.get("invert back") > 0 && forward < 0){
            turn = -turn;
        }

        double[] powers = cheesyDrive(forward, turn, isQuickTurn, false);

        hardware.drivetrain.setPowers(powers[0], powers[1]);

        hardware.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.shooter.setVelocity(gamepad2.right_trigger);

        telemetry.addData("forward", forward);
        telemetry.addData("turn", turn);
        telemetry.addData("leftPower", powers[0]);
        telemetry.addData("rightPower", powers[1]);
        telemetry.addData("isQuickTurn", isQuickTurn);
        telemetry.update();
    }


    public double[] cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {

        final double kThrottleDeadband = 0.05;
        final double kWheelDeadband = 0.01;


        // These factor determine how fast the wheel traverses the "non linear" sine curve.
        final double kHighWheelNonLinearity = 0.65;
        final double kLowWheelNonLinearity = 0.5;

        final double kHighNegInertiaScalar = 4.0;

        final double kLowNegInertiaThreshold = 0.65;
        final double kLowNegInertiaTurnScalar = 3.5;
        final double kLowNegInertiaCloseScalar = 4.0;
        final double kLowNegInertiaFarScalar = 5.0;


        final double kHighSensitivity = 0.65;
        final double kLowSensitiity = 0.65;


        final double kQuickStopDeadband = 0.5;
        final double kQuickStopWeight = 0.1;
        final double kQuickStopScalar = 5.0;


        double mOldWheel = 0.0;
        double mQuickStopAccumlator = 0.0;
        double mNegInertiaAccumlator = 0.0;


        wheel = -handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);


        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;


        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kLowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > kLowNegInertiaThreshold) {
                    negInertiaScalar = kLowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kLowNegInertiaCloseScalar;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;


        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;


        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }


        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;


        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        return new double[] { leftPwm, rightPwm };
    }


    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }


    public double limit(double val, double limit) {
        if (val > limit) {
            return limit;
        } else if (val < -limit) {
            return -limit;
        } else {
            return val;
        }
    }
}
