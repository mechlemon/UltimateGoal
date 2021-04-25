package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode._Hardware;

import java.util.ArrayList;

public class AutoSequence {

    public _Hardware hardware;
    public ArrayList<AbstractAction> actions;
    public int actionIndex;
    public AbstractAction action;
    public boolean done;

    public AutoSequence(_Hardware hardware, AbstractAction... actions_arr){
        this.hardware = hardware;

        //convert array into arraylist
        actions = new ArrayList<AbstractAction>();
        for(AbstractAction action_new : actions_arr){
            actions.add(action_new);
        }

        actionIndex = 0;
    }

    public void addAction(AbstractAction action_new){
        actions.add(action_new);
    }

    public void runSequence(){

        if(actionIndex > actions.size() - 1){
            hardware.stop();
            done = true;
            return;
        }

        action = actions.get(actionIndex);
        if(!action.done){
            action.runAction(hardware);
        }else{
            actionIndex++;
        }
    }

}
