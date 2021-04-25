
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.VuforiaPhone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


//makes a robot, including motors, servos, imu
public class _Hardware {

    public _Drivetrain drivetrain;

    public IMU imu;

    public DcMotorEx shooter;

    public _FlippyThread flippy;

    public DcMotorEx intake;

    public Servo ostrichServo; //pushes rings into shooter to shoot

    public _Hardware(HardwareMap hardwareMap, Telemetry telemetry, boolean justDrivetrain){

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();

        drivetrain = new _Drivetrain(
                hardwareMap.get(DcMotorEx.class, "2-0"),    //left
                hardwareMap.get(DcMotorEx.class, "2-1"),    //right
                hardwareMap.get(DcMotorEx.class, "2-2"),     //odo
                imu
        );

        if(!justDrivetrain){
            shooter = hardwareMap.get(DcMotorEx.class, "1-3");
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            flippy = new _FlippyThread(
                hardwareMap.get(DcMotorEx.class, "1-0"),     //lower left joint
                hardwareMap.get(DcMotorEx.class, "1-1"),     //lower right joint
                hardwareMap.get(DcMotorEx.class, "1-2")     //top joint
            );

            intake = hardwareMap.get(DcMotorEx.class, "2-3");

            ostrichServo = hardwareMap.servo.get("1-0s");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

    double ostrichRestPos = 0.63;
    double ostrichShootPos = 0.70;
    double ostrichIntakePos = 0.3;

    public void ostrichRest(){
        ostrichServo.setPosition(ostrichRestPos);
    }

    public void ostrichShoot(){
        ostrichServo.setPosition(ostrichShootPos);
    }

    public void ostrichIntake(){
        ostrichServo.setPosition(ostrichIntakePos);
    }

    public void stop(){
        drivetrain.setPowers(0,0);
        shooter.setPower(0);
        intake.setPower(0);
        flippy.stop();
    }





}