
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.IMU;
import org.firstinspires.ftc.teamcode.lib.VuforiaPhone;


//makes a robot, including motors, servos, imu
public class _Hardware {


    public _Drivetrain drivetrain;
    public IMU imu;
//    public DcMotor shooter;



    public _Hardware(HardwareMap hardwareMap, Telemetry telemetry){

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();


        drivetrain = new _Drivetrain(
            hardwareMap.get(DcMotorEx.class, "1-0"),    //left
            hardwareMap.get(DcMotorEx.class, "1-1"),    //right
            hardwareMap.get(DcMotorEx.class, "1-2"),     //odo
            imu
        );

//        shooter = hardwareMap.get(DcMotorEx.class, "1-2");


        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }



}