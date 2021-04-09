package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

public class _Drivetrain {

    private final double WHEEL_CIRCUMFERENCE = 1.88976 * Math.PI; //48mm diameter omni to inches
    private final double ODO_GEAR_RATIO = 40/60.0;
    private final double ODO_TICKS_PER_REV = 8196;

    public final double TRACK_WIDTH = 16; //distance between wheels, inches

    private final double ODO_TICKS_PER_INCH;

    public DcMotorEx leftMotor, rightMotor, horizOdoMotor;
    public double leftZero, rightZero, horizOdoZero = 0;

    public IMU imu;

    public Pose2D robotPose = new Pose2D();

    public _Drivetrain(DcMotorEx leftMotor, DcMotorEx rightMotor, DcMotorEx horizOdoMotor, IMU imu) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.horizOdoMotor = horizOdoMotor;

        this.imu = imu;

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ODO_TICKS_PER_INCH = ODO_TICKS_PER_REV * ODO_GEAR_RATIO / WHEEL_CIRCUMFERENCE;
    }

    public void setPowers(double leftPower, double rightPower) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setPower(Range.clip(leftPower, -1, 1));
        rightMotor.setPower(Range.clip(rightPower, -1, 1));

        updateOdo();
    }

    public void setVelocities(double leftVelo, double rightVelo){
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(Range.clip(leftVelo, -1, 1)); //as a proportion of max speed
        rightMotor.setPower(Range.clip(rightVelo, -1, 1));

        updateOdo();
    }

    double lastLDist, lastRDist, lastHDist = 0;
    double lastHeading = 0;
    public void updateOdo(){
        Pose2D step = calcStepSeparate(
                getLeftDistance() - lastLDist,
                getRightDistance() - lastRDist,
                getHorizDistance() - lastHDist,
                imu.getHeading() - lastHeading
        );

        lastLDist = getLeftDistance();
        lastRDist = getRightDistance();
        lastHDist = getHorizDistance();

        robotPose = robotPose.exp(step);
        robotPose.ang = imu.getHeading();
    }

    Pose2D calcStepSeparate(double dL, double dR, double dH, double dtheta){
        double r = TRACK_WIDTH * 0.5;

        double possiblyZeroOverZero;
        if(Math.abs(dR - dL) > 1e-3){
            possiblyZeroOverZero = Math.sin((dR - dL) / (2*r)) / (dR - dL);
        }else{
            //Maclaurin approximation
            possiblyZeroOverZero = 1/(2*r) + (dR - dL)*(dR - dL) / (48*r*r*r);
        }

        double forward = 2*r * (dL + dR) * possiblyZeroOverZero;
        return new Pose2D(forward, dH, dtheta);
    }

    public double getLeftDistance(){
        return (leftMotor.getCurrentPosition() - leftZero) / ODO_TICKS_PER_INCH;
    }

    public double getRightDistance(){
        return (rightMotor.getCurrentPosition() - rightZero) / ODO_TICKS_PER_INCH;
    }

    public double getHorizDistance(){
        return (horizOdoMotor.getCurrentPosition() - horizOdoZero) / ODO_TICKS_PER_INCH;
    }

    public void resetEncoders(){
        leftZero = leftMotor.getCurrentPosition();
        rightZero = rightMotor.getCurrentPosition();
        horizOdoZero = horizOdoMotor.getCurrentPosition();
    }
}
