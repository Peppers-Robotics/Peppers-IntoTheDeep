package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;

@Config
public class Extension {
    public static ServoPlus servo;
    public static double extendoPos = 120, retractPos = 319;
    public static LinearFunction f = new LinearFunction(retractPos, extendoPos);
    public static void Retract(){
        servo.setAngle(retractPos);
    }
    public static void Extend(double precent){
        servo.setAngle(f.getOutput(precent));
    }
}
