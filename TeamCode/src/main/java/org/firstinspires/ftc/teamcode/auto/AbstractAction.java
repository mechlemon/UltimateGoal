package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode._Hardware;

public abstract class AbstractAction {
    public boolean done = false;
    public abstract void runAction(_Hardware hardware);
}