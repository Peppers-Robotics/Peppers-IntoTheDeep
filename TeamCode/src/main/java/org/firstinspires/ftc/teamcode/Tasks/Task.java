package org.firstinspires.ftc.teamcode.Tasks;

import kotlin.jvm.internal.Lambda;

public abstract class Task {
    public boolean needsReset = false;
    public abstract boolean Run();
    public void reset(){}
}

