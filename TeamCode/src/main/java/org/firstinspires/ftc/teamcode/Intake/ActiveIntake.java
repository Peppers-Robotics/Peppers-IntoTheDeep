package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

@Config
public class ActiveIntake {
    public static CachedMotor motor;

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
        motor.setPower(power);
    }
    public static boolean isOff(){
        return Math.abs(motor.getPower()) < 0.1;
    }
}
