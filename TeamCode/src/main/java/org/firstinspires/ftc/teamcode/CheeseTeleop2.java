package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.ButtonPress;
import org.firstinspires.ftc.teamcode.lib.Tuner;


//@TeleOp(name = "CheeseTeleop2", group = "drive")
public class CheeseTeleop2 extends OpMode {


    private _Hardware hardware;
    private Tuner tuner;

    private String[] titles = new String[] {"turnCoeff", "", "quickturn cutoff", "singlestick", "invert back"};
    private double[] values = new double[] {     0.35  ,          3        ,      1       ,       1      };

    private ButtonPress shiftPress = new ButtonPress();
    private boolean isHighGear = true;

    public void init(){
        hardware = new _Hardware(hardwareMap, telemetry);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    public void loop(){
        tuner.tune();

        double forward = -gamepad1.right_stick_y;

        double turn;
        if(tuner.get("singlestick") > 0){
            turn = -gamepad1.right_stick_x * tuner.get("turnCoeff");
        }else{
            turn = -gamepad1.left_stick_x * tuner.get("turnCoeff");
        }

        boolean isQuickTurn = Math.abs(turn/forward) > tuner.get("quickturn cutoff");

        if(!isQuickTurn && tuner.get("invert back") > 0 && forward < 0){
            turn = -turn;
        }

        if(shiftPress.status(gamepad1.right_bumper) == ButtonPress.Status.COMPLETE){
            if(isHighGear){
                isHighGear = false;
            }else{
                isHighGear = true;
            }
        }

        double[] powers = cheesyDrive(forward, turn, isQuickTurn, isHighGear);

        hardware.drivetrain.setPowers(powers[0], powers[1]);

        telemetry.addData("forward", forward);
        telemetry.addData("turn", turn);
        telemetry.addData("leftPower", powers[0]);
        telemetry.addData("rightPower", powers[1]);
        telemetry.addData("isQuickTurn", isQuickTurn);
        telemetry.addData("isHighGear", isHighGear);
        telemetry.update();
    }


    public static double[] cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {

        final double kThrottleDeadband = 0.05;
        final double kWheelDeadband = 0.01;


        // These factor determine how fast the wheel traverses the "non linear" sine curve.
        final double kHighWheelNonLinearity = 0.65;
        final double kLowWheelNonLinearity = 0.5;

        final double sensitivity = 0.65;


        wheel = -handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);


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

        double angularPower;

        // Quickturn!
        if (isQuickTurn) {
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity;
        }


        rightPwm = leftPwm = throttle;
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


    public static double handleDeadband(double val, double deadband) {
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
