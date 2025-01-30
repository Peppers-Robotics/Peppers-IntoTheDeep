package org.firstinspires.ftc.teamcode.Tasks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Wait extends Task implements Cloneable {
    private long wait;
    private long track;
    public Wait(double sec){
        wait = (long) (sec * 1000);
        track = -1;
    }
    @Override
    public boolean Run() {
        if(track == -1){
            track = System.currentTimeMillis();
            return false;
        }
        boolean r = (System.currentTimeMillis() - track) >= wait;
        if(r) track = -1;
        return r;
//        Robot.telemetry.addData("time", time.seconds());
    }
}
