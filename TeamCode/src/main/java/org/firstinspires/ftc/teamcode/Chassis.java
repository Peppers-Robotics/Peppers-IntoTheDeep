package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;

@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static double rFL = -1, rFR = 1, rBL = -1, rBR = 1;
    public static void drive(double x, double y, double h){
        double div = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(h), 1);
        double pow = 1;
        if(Controls.gamepad1.gamepad.cross) pow = 0.7;

        FL.setPower((y + x + h) / div * rFL * pow);
        FR.setPower((y - x - h) / div * rFR * pow);
        BL.setPower((y - x + h) / div * rBL * pow);
        BR.setPower((y + x - h) / div * rBR * pow);
    }
}
