package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

@Config
public class ActiveIntake {
    public static CachedMotor motor;
    public static ServoPlus blocker;
    public static double block = 180, unblock = 265;

    public static void Block(){
        blocker.setAngle(block);
    }
    public static void Unblock(){
        blocker.setAngle(unblock);
    }
    public static boolean isBlocked(){
        return blocker.isEqualToAngle(block);
    }

    public static void powerOn(){
        powerOn(1);
    }
    public static void powerOn(double pow){
        motor.setPower(-pow);
    }
    public static void powerOff(){
        motor.setPower(0);
    }
    public static void Reverse(double power){
        if(power > 0.6) power = 0.6;
        motor.setPower(power);
    }
    public static boolean isOff(){
        return Math.abs(motor.getPower()) < 0.1;
    }
}
