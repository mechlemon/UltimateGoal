package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ThreadTest", group = "test")

public class ThreadTest extends OpMode {

    ModuleThread motorThread;

    DcMotorEx motor_main;

    int count = 0;
    double power = 0;
    double speed = 0.9;

    double lastPos_main = 0;
    double lastVelo_main = 0;
    long lasttime;

    boolean done = false;

    @Override
    public void init() {
//        motorThread = new ModuleThread("motorThread");
        motor_main = hardwareMap.get(DcMotorEx.class, "1-2");

    }

    @Override
    public void loop() {
//        power = -gamepad1.left_stick_y;
        double dt = System.nanoTime() - lasttime;
        lasttime = System.nanoTime();

        double velo = (motor_main.getCurrentPosition() - lastPos_main) / (dt * 1e-9);
        lastPos_main = motor_main.getCurrentPosition();

        motor_main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_main.setPower(speed);

        if(Math.abs(velo - lastVelo_main) < 100 && velo > 100){
//            speed = 0;
            done = true;
        }else{
            if(true){
                count++;
            }
        }
        lastVelo_main = velo;

//        telemetry.addData( "movements thread (fullpower)", motorThread.threadTasks.movements);
        telemetry.addData( "count", count);
        telemetry.addData( "velo", velo);
        telemetry.addData( "velo - lastVelo_main", velo - lastVelo_main);
        telemetry.update();

    }

    @Override
    public void stop(){
//        motorThread.stop();
    }

    public class ModuleThread implements Runnable{

        ThreadTasks threadTasks;

        private boolean exit;
        Thread t;
        ModuleThread(String name) {
            threadTasks = new ThreadTasks(name);
            t = new Thread(this, name);
            exit = false;
            t.start();
        }

        public void run(){
            threadTasks.initialize();
            while(!exit) {
                threadTasks.execute();
            }
        }

        public void stop(){
            threadTasks.stop();
            exit = true;
        }
    }

    class ThreadTasks {
        int lastPosition = 0;
        String name;

        int movements = 0;
        DcMotor motor;

        ThreadTasks(String name_input){
            name = name_input;
        }
        void initialize(){
            motor = hardwareMap.get(DcMotor.class, "1-3");
        }

        void execute(){
            motor.setPower(power);
            if(motor.getCurrentPosition() != lastPosition){
                movements++;
            }
            lastPosition = motor.getCurrentPosition();
        }

        void stop(){
            motor.setPower(0);
        }
    }


}
