package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

@Config
public class Extendo {
    public static CachedMotor motor;
    public static int MaxExtension = 1350;

    public static PIDController pidController = new PIDController(0.15, 0, 0.0006);

    public static boolean AutoAdjustByPID = false;

    public static void SetRawPower(double p){
        AutoAdjustByPID = false;
        motor.setPower(p);
    }
    public static void SetTargetPosition(double p){
        AutoAdjustByPID = true;
        pidController.setTargetPosition(p);
    }
    public static double GetCurrentPositon(){
        return motor.getCurrentPosition();
    }

    public static void Update(){
        if(AutoAdjustByPID) {
            motor.setPower(pidController.calculatePower(GetCurrentPositon()));
        }
    }

}
