package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;

@Config
public class Extension {
    public static ServoPlus servo;
    public static double extendoPos = 95, retractPos = 295;
    public static LinearFunction f = new LinearFunction(retractPos, extendoPos);
    private static double frac = 0;
    public static void Retract(){
        servo.setAngle(retractPos);
    }
    public static void Extend(double precent){
        if(precent > 1) precent = 1;
        if(precent < 0) precent = 0;
        frac     = precent;
        servo.setAngle(f.getOutput(precent));
    }
    public static double getPrecent(){
        return frac;
    }
}
