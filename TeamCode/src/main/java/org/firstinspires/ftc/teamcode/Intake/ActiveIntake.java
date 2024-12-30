package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;

@Config
public class ActiveIntake {
    public static CachedMotor intake;
    public static void PowerOn(){
        intake.setPower(1);
    }
    public static void Reverse(){
        intake.setPower(-1);
    }
    public static void PowerOff(){
        intake.setPower(0);
    }
}
