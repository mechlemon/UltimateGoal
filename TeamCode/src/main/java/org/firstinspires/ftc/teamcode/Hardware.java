package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.VuforiaPhone;


//makes a robot, including motors, servos, imu
public class Hardware {


    public Drivetrain drivetrain;
    public IMU imu;
    public VuforiaPhone vuforiaPhone;
    public DcMotorEx shooter;



    public Hardware(HardwareMap hardwareMap, Telemetry telemetry, boolean initIMU, boolean initVuforia){
        drivetrain = new Drivetrain(hardwareMap.get(DcMotor.class, "1-0"),
                                    hardwareMap.get(DcMotor.class, "1-1"));

        shooter = hardwareMap.get(DcMotorEx.class, "1-2");

        if(initIMU){
            imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
            imu.setHeadingAxis(IMU.HeadingAxis.YAW);
            imu.initialize();
        }


        if(initVuforia){
            vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }



}