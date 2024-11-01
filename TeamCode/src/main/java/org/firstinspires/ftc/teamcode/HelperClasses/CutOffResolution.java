package org.firstinspires.ftc.teamcode.HelperClasses;

public class CutOffResolution {
    public static double GetResolution(double x, double decimals){
        long p10 = (long) Math.pow(10, decimals);
        return ((int)(x * p10) / (double)p10);
    }
}
