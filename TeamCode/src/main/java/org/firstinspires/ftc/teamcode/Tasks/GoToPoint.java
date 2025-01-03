package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Pose2D;

public class GoToPoint extends Task {
    private final Pose2D pose;
    private boolean block = true;
    public GoToPoint(Pose2D point){
        pose = point;
    }
    public GoToPoint(Pose2D point, boolean block){
        pose = point;
        this.block = block;
    }
    @Override
    public boolean Run() {
        if(!block){
            Chassis.SetTargetPosition(pose);
            return true;
        }
        Chassis.SetTargetPosition(pose);
        return Chassis.DistanceToTarget() < 0.5;
    }
}
