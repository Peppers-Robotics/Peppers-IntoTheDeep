package com.ftc.bumblebee.Localizers;

public class Pose2d {
    public double x, y, h;
    public static double NormalizeRadians(double raw){
        while(raw > Math.PI) raw -= 2 * Math.PI;
        while(raw < -Math.PI) raw += 2 * Math.PI;
        return raw;
    }
    public Pose2d(double x, double y, double h){
        h = NormalizeRadians(h);
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose2d(double x, double y){
        this.x = x;
        this.y = y;
        h = 0;
    }
}
