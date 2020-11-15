package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain {

    private final double WHEEL_CIRCUMFERENCE = 1.88976 * Math.PI; //48mm diameter omni to inches
    private final double ODO_GEAR_RATIO = 40/60.0;
    private final double ODO_TICKS_PER_REV = 8196;

    private final double ODO_TICKS_PER_INCH;

    public DcMotor leftMotor, rightMotor;
    private double leftZero, rightZero = 0;

    public Drivetrain(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ODO_TICKS_PER_INCH = ODO_TICKS_PER_REV * ODO_GEAR_RATIO / WHEEL_CIRCUMFERENCE;
    }

    public void setPowers(double leftPower, double rightPower) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setPower(Range.clip(leftPower, -1, 1));
        rightMotor.setPower(Range.clip(rightPower, -1, 1));
    }

    public void setVelocities(double leftVelo, double rightVelo){
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(Range.clip(leftVelo, -1, 1)); //as a proportion of max speed
        rightMotor.setPower(Range.clip(rightVelo, -1, 1));
    }

    public double getLeftDistance(){
        return (leftMotor.getCurrentPosition() - leftZero) / ODO_TICKS_PER_INCH;
    }

    public double getRightDistance(){
        return (rightMotor.getCurrentPosition() - rightZero) / ODO_TICKS_PER_INCH;
    }

    public void resetEncoders(){
        leftZero = leftMotor.getCurrentPosition();
        rightZero = rightMotor.getCurrentPosition();
    }
}
