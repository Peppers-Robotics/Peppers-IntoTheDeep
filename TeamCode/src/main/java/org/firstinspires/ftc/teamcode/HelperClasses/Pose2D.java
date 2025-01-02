package org.firstinspires.ftc.teamcode.HelperClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Intake.IntakeController;

public class Pose2D {
    public double x, y, h;
    public Pose2D(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose2D(double x, double y){
        this.x = x;
        this.y = y;
        this.h = 0;
    }
    public Pose2D(double x){
        this.x = x;
        this.y = 0;
        this.h = 0;
    }
    public Pose2D(){
        this.x = 0;
        this.y = 0;
        this.h = 0;
    }

    public Pose2D(Pose2d p) {
        x = p.getX();
        y = p.getY();
        h = p.getHeading();
    }
    @NonNull
    public String toString(){
        return x + " " + y + " " + h;
    }
}
