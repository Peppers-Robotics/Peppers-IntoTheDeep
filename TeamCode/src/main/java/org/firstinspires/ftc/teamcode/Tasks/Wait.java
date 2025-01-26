package org.firstinspires.ftc.teamcode.Tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Wait extends Task {
    private ElapsedTime time;
    private double s;
    public Wait(double sec){
        s = sec;
        time = new ElapsedTime();
        time.reset();
        needsReset = true;
    }
    @Override
    public void reset(){ time.reset(); needsReset = false;}
    @Override
    public boolean Run() {
//        Robot.telemetry.addData("time", time.seconds());
        if(time.seconds() >= s){
            needsReset = true;
            return true;
        }
        return time.seconds() >= s;
    }
}
