
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

//    public Servo yeetup; //pushes rings into shooter to shoot
//    public Servo spatula; //scoops rings up to shooter to load

    // shooter constants
    public double shooterDelay = 0.3;
    public double shooterBackpos = 0.79;
    public double shooterFrontpos = 0.69;

    public double currentRPM;

    public _Hardware(HardwareMap hardwareMap, Telemetry telemetry){

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();

        drivetrain = new _Drivetrain(
            hardwareMap.get(DcMotorEx.class, "2-0"),    //left
            hardwareMap.get(DcMotorEx.class, "2-1"),    //right
            hardwareMap.get(DcMotorEx.class, "2-2"),     //odo
            imu
        );

        shooter = hardwareMap.get(DcMotorEx.class, "1-3");

//        yeetup = hardwareMap.servo.get("yeetup");
//        spatula = hardwareMap.servo.get("spatula");

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

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

//            yeetup = hardwareMap.servo.get("yeetup");
//            spatula = hardwareMap.servo.get("spatula");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

    void setShooterRPM(double rpm){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentRPM = shooter.getVelocity(AngleUnit.RADIANS) * 60.0 / (2 * Math.PI);
        shooter.setVelocity(rpm * 2 * Math.PI / 60.0, AngleUnit.RADIANS);
    }

    void shoot(int numShots, double runtime){
        if(runtime < numShots * shooterDelay){

//            if(runtime % shooterDelay < 0.5 * shooterDelay){
//                yeetup.setPosition(shooterFrontpos);
//            }else{
//                yeetup.setPosition(shooterBackpos);
//            }
        }
    }

    void stop(){
        drivetrain.setPowers(0,0);
        shooter.setPower(0);
    }





}