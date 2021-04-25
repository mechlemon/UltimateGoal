package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Util.PID;

import static java.lang.Math.toRadians;


public class _FlippyUnthread{

    public DcMotorEx armLow1Motor, armLow2Motor, armTopMotor;
    public PID armLow1PID = new PID();
    public PID armLow2PID = new PID();
    public PID armTopPID = new PID();

    public double armLow1Angle, armLow2Angle, armTopAngle;

    public double lowSetPos, topSetPos = 0;

    public double K_armLowMaxPower = 0.7;
    public double K_armTopMaxPower = 0.9;
    public double K_antigravLow = 0.1;
    public double K_antigravTop = 0.16;
    public double K_antibacklash = 0.1;

    public double K_lowP = 0.5;
    public double K_lowI = 0;
    public double K_lowD = 0;
    public double K_lowFx = 0.08;

    public double K_topP = 0.5;
    public double K_topI = 0;
    public double K_topD = 0;
    public double K_topFx = 0.08;

    public double hertz = 0;
    public double lastTime;

    public _FlippyUnthread(DcMotorEx armLow1Motor, DcMotorEx armLow2Motor, DcMotorEx armTopMotor) {
        this.armLow1Motor = armLow1Motor;
        this.armLow2Motor = armLow2Motor;
        this.armTopMotor = armTopMotor;

        this.armLow1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armLow2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armLow2Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.armTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armLow2Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTime = System.nanoTime();
    }


    public void set(double lowSetPos, double topSetPos){
        this.lowSetPos = lowSetPos;
        this.topSetPos = topSetPos;
    }

    public void execute(){

        double dt = (System.nanoTime() - lastTime) * 1e-9;
        lastTime = System.nanoTime();
        hertz = 1. / dt;

        armLow1Angle = (armLow1Motor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (36.0/16.0));
        armLow2Angle = (armLow1Motor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (36.0/16.0));
        double antigravLow = K_antigravLow * Math.cos(armLow1Angle);

        armTopAngle = (armTopMotor.getCurrentPosition() / 28.0) * 2*Math.PI / (50.9 * (34.0/16.0)) + armLow1Angle;
        double antigravTop = K_antigravTop * Math.cos(armTopAngle);

        armLow1PID.setkP(K_lowP);
        armLow1PID.setkI(K_lowI, 0.2, 0.5);
        armLow1PID.setkD(K_lowD);
        armLow1PID.setkFx(K_lowFx, toRadians(3));
        armLow2PID.copyConstants(armLow1PID);

        armTopPID.setkP(K_topP);
        armLow1PID.setkI(K_topI, 0.2, 0.5);
        armLow1PID.setkD(K_topD);
        armTopPID.setkFx(K_topFx, toRadians(3));

        armLow1PID.loop(armLow1Angle, lowSetPos, dt);
        armLow2PID.loop(armLow2Angle, lowSetPos + K_antibacklash, dt);
        armTopPID.loop(armTopAngle, topSetPos, dt);

        if(lowSetPos > toRadians(30) && armLow1Angle < toRadians(30)){
            K_armTopMaxPower = 0;
        }

        armLow1Motor.setPower(Range.clip(armLow1PID.getPower(), -K_armLowMaxPower, K_armLowMaxPower) + antigravLow);
        armLow2Motor.setPower(Range.clip(armLow2PID.getPower(), -K_armLowMaxPower, K_armLowMaxPower) + antigravLow);
        armTopMotor.setPower(Range.clip(armTopPID.getPower(), -K_armTopMaxPower, K_armTopMaxPower) + antigravTop);



    }


    void stop(){
        armLow1Motor.setPower(0);
        armLow2Motor.setPower(0);
        armTopMotor.setPower(0);
    }
}


