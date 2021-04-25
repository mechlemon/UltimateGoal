package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode._Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector2D;

public class LineAction extends AbstractAction{

    Vector2D endpoint;
    double speed;
    double toleranceRadius;

    public LineAction(Vector2D endpoint, double speed, double toleranceRadius){
        this.endpoint = endpoint;
        this.speed = speed;
        this.toleranceRadius = toleranceRadius;
    }

    @Override
    public void runAction(_Hardware hardware) {
        if(hardware.drivetrain.robotPose1.dist(endpoint) > toleranceRadius){
            hardware.drivetrain.nyoomToPoint(endpoint, speed);
        }else{
            done = true;
        }
    }


    public String toString(){
        return "endpoint: " + endpoint + ", speed: " + speed + ", toleranceRadius: " + toleranceRadius;
    }

}
