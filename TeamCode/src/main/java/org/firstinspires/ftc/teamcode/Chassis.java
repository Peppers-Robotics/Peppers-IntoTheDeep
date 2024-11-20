package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;

@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static double rFL = 1, rFR = -1, rBL = 1, rBR = -1;
    public static void drive(double x, double y, double h){
        double div = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(h), 1);

        FL.setPower((y + x + h) / div * rFL);
        FR.setPower((y - x - h) / div * rFR);
        BL.setPower((y - x + h) / div * rBL);
        BR.setPower((y + x - h) / div * rBR);
    }
}
