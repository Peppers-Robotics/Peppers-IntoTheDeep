package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Chassis;

public class GoToPoint extends Task {
    private Pose2d pose;
    public GoToPoint(Pose2d point){
        pose = point;
    }
    @Override
    public boolean Run() {
        if(Chassis.GetTargetPosition() != pose) Chassis.SetTargetPosition(pose);
        return Chassis.DistanceToTarget() < 0.5;
    }
}
