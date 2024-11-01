package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;

public class ActiveIntake {
    public static CachedMotor motor;

    synchronized public static void powerOn(){
        motor.setPower(1);
    }
    synchronized public static void powerOff(){
        motor.setPower(0);
    }
    synchronized public static void Reverse(){
        motor.setPower(-1);
    }
}
