package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.Util.PID;
import org.firstinspires.ftc.teamcode.lib.Vector2D;

public class _Drivetrain {

    private final double WHEEL_CIRCUMFERENCE = 1.88976 * Math.PI; //48mm diameter omni to inches
    private final double ODO_GEAR_RATIO = 40/60.0;
    private final double ODO_TICKS_PER_REV = 8196;

    public final double TRACK_WIDTH = 15.75; //distance between wheels, inches

    private final double ODO_TICKS_PER_INCH;

    public DcMotorEx leftMotor, rightMotor, horizOdoMotor;
    public double leftZero, rightZero, horizOdoZero = 0;

    public IMU imu;

    public Pose2D robotPose1 = new Pose2D();
    public Pose2D robotPose2 = new Pose2D();

    public boolean reversed = false;
    public PID turnPID = new PID();
    public PID drivePID = new PID();

    public double maxForwardVel = 12;

    public _Drivetrain(DcMotorEx leftMotor, DcMotorEx rightMotor, DcMotorEx horizOdoMotor, IMU imu) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.horizOdoMotor = horizOdoMotor;

        this.imu = imu;

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ODO_TICKS_PER_INCH = ODO_TICKS_PER_REV * ODO_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

        turnPID.setkP(1);
        drivePID.setkP(0.5);
    }

    public void fieldOriented(double vx, double vy, double extraspin){
        Vector2D linVel = new Vector2D(vx, vy, Vector2D.Type.CARTESIAN);
        if(linVel.getMagnitude() < 0.01 && Math.abs(extraspin) < 0.01){
            setPowers(0,0);
            return;
        }

        double dt = 0.02;

        double targetAngle = calcClosestAngle180(imu.getHeading(), linVel.getAngle());
        double turnPower = turnPID.loop(imu.getHeading(), targetAngle, dt);

        if(Math.abs(extraspin) > 0.01){
            turnPower = extraspin;
        }

        double targetVelo = linVel.getMagnitude();
        if(reversed) targetVelo *= -1;
        double drivePower = 0.8 * targetVelo;

        double leftVelo = drivePower - turnPower;
        double rightVelo = drivePower + turnPower;

        double maxVelo = Math.max(leftVelo, rightVelo); //to keep the ratio between L and R velo
        if(maxVelo > 1){
            leftVelo /= maxVelo;
            rightVelo /= maxVelo;
        }

        setPowers(leftVelo, rightVelo);
    }

    public void drive(double drivePower, double turnPower){

        double leftVelo = drivePower - turnPower;
        double rightVelo = drivePower + turnPower;

        double maxVelo = Math.max(leftVelo, rightVelo); //to keep the ratio between L and R velo
        if(maxVelo > 1){
            leftVelo /= maxVelo;
            rightVelo /= maxVelo;
        }

        setPowers(leftVelo, rightVelo);
    }

    double calcClosestAngle180(double currentAngle, double targetAngle){
        double differencePi = (currentAngle - targetAngle) % Math.PI; //angle error from (-180, 180)

        double closestAngle;
        if(Math.abs(differencePi) < (Math.PI / 2.0)){ //chooses closer of the two acceptable angles closest to currentAngle
            closestAngle = currentAngle - differencePi;
        }else{
            closestAngle = currentAngle - differencePi + Math.copySign(Math.PI, differencePi);
        }

        double difference2Pi = (closestAngle - targetAngle) % (2 * Math.PI);
        reversed = Math.abs(difference2Pi) > (Math.PI / 2.0); //if the difference is closer to 180, reverse direction

        return closestAngle;
    }

    private double calcClosestAngle360(double currentAngle, double targetAngle){
        double diffNormalized = Util.normalizeAngle(currentAngle - targetAngle, Math.PI); //angle error from (-PI, PI)
        return currentAngle - diffNormalized;
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

    double lastLDist, lastRDist, lastHDist = 0;
    double lastHeading = 0;
    public void updateOdo(){

        double currLDist = getLeftDistance();
        double currRDist = getRightDistance();
        double currHDist = getHorizDistance();
        double currHeading = imu.getHeading();

        Pose2D step1 = calcStepAvg(
                currLDist - lastLDist,
                currRDist - lastRDist,
                currHDist - lastHDist,
                currHeading - lastHeading
        );

        Pose2D step2 = calcStepArcSeparate(
                currLDist - lastLDist,
                currRDist - lastRDist,
                currHDist - lastHDist,
                currHeading - lastHeading
        );



        robotPose1 = robotPose1.exp(step1.rotateVec(lastHeading));
        robotPose2 = robotPose2.exp(step2.rotateVec(lastHeading));
        robotPose1.ang = imu.getHeading();
        robotPose2.ang = imu.getHeading();

        lastLDist = currLDist;
        lastRDist = currRDist;
        lastHDist = currHDist;
        lastHeading = currHeading;
    }

    Pose2D calcStepArcSeparate(double dL, double dR, double dH, double dtheta){
        double r = TRACK_WIDTH * 0.5;

        double possiblyZeroOverZero;
        if(Math.abs(dR - dL) > 1e-3){
            possiblyZeroOverZero = Math.sin((dR - dL) / (2*r)) / (dR - dL);
        }else{
            //Maclaurin approximation
            possiblyZeroOverZero = 1/(2*r) - (dR - dL)*(dR - dL) / (6 * (2*r)*(2*r));
        }

        double forward = r * (dL + dR) * possiblyZeroOverZero;
        return new Pose2D(forward, dH, dtheta);
    }

    Pose2D calcStepAvg(double dL, double dR, double dH, double dtheta){
        double forward = 0.5 * (dL + dR);
        return new Pose2D(forward, dH, dtheta);
    }

    public Pose2D getRobotCenter(){
        return robotPose2.add(new Vector2D(10, 0, Vector2D.Type.CARTESIAN).rotate(robotPose2.getAngle()));
    }

    public double getLeftDistance(){
        return -(leftMotor.getCurrentPosition() - leftZero) / ODO_TICKS_PER_INCH;
    }

    public double getLeftVelocity(){
        return -leftMotor.getVelocity(AngleUnit.RADIANS);
    }

    public double getRightDistance(){
        return (rightMotor.getCurrentPosition() - rightZero) / ODO_TICKS_PER_INCH;
    }

    public double getRightVelocity(){
        return rightMotor.getVelocity(AngleUnit.RADIANS);
    }

    public double getAvgForwardDist(){
        return 0.5*(getLeftDistance() + getRightDistance());
    }

    public double getAvgForwardVelo(){
        return 0.5*(getLeftVelocity() + getRightVelocity());
    }

    public double getHorizDistance(){
        return (horizOdoMotor.getCurrentPosition() - horizOdoZero) / (ODO_TICKS_PER_INCH / ODO_GEAR_RATIO);
    }

    public void reset(){
        leftZero = leftMotor.getCurrentPosition();
        rightZero = rightMotor.getCurrentPosition();
        horizOdoZero = horizOdoMotor.getCurrentPosition();
        imu.resetHeading();

        lastLDist = 0;
        lastRDist = 0;
        lastHDist = 0;
        lastHeading = 0;

        robotPose1 = new Pose2D();
        robotPose2 = new Pose2D();
    }
}
