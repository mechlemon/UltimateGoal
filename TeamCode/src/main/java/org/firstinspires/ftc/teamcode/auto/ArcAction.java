package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode._Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector2D;

public class ArcAction extends AbstractAction{

    Vector2D center;
    double speed;
    double angleChange;

    Double startAngle;

    AngleTracker angleTracker = new AngleTracker(-Math.PI, Math.PI);

    public ArcAction(Vector2D center, double speed, double angleChange){
        this.center = center;
        this.speed = speed;
        this.angleChange = angleChange;
    }


    @Override
    public void runAction(_Hardware hardware) {


        if(startAngle == null){
            startAngle = hardware.imu.getHeading();
        }

        if(Math.abs(hardware.imu.getHeading() - startAngle) < Math.abs(angleChange)){
            hardware.drivetrain.nyoomAboutPoint(center, speed * Math.signum(angleChange));

        }else{
            done = true;
        }
    }

    public String toString(){
        return "center: " + center + ", speed: " + speed + ", angleChange: " + angleChange;
    }

}