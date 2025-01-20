package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

@Config
public class DropDown {
    public static ServoPlus DropDownLeft;
    public static AsymmetricMotionProfile profile = new AsymmetricMotionProfile(5000, 50000, 50000);
    public static double UpPosition = 275, MiddlePosition = 173, DownPosition = 130;
    public static double rangeRetract = 170, rangeExtend = 145;
    public static double instantPosition = UpPosition, step = 15;

    public static void GoUp(){
        profile.startMotion(profile.getPosition(), UpPosition);
        instantPosition = UpPosition;
    }
    public static void GoMiddle(){
        profile.startMotion(profile.getPosition(), MiddlePosition);
    }
    public static void GoDown(){
        profile.startMotion(profile.getPosition(), DownPosition);
        instantPosition = DownPosition;
    }

    public static boolean isUp(){
        return Math.abs(profile.getPosition() - UpPosition) <= 1.5f;
    }
    public static void setInstantPosition(double k){
        double b = rangeRetract;
        double range = 700;
        double a = (-rangeExtend + rangeRetract) / range;
        double down = 0;
        if(OutTakeStateMachine.inAuto)
            down = DownPosition;
        else
            down = a * Extendo.getCurrentPosition() + b;
        instantPosition = UpPosition - Math.abs(down - UpPosition) * k;
        profile.setInstant(instantPosition);
    }

    public static boolean isMiddle(){
        return Math.abs(profile.getPosition() - MiddlePosition) <= 1.5f;
    }

    public static boolean isDown(){
        return Math.abs(profile.getPosition() - DownPosition) <= 1.5f;
    }
    public synchronized static void Update(){
        DropDownLeft.setAngle(profile.getPosition());
        profile.update();
    }
}
