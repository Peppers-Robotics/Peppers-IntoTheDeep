package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

public class ActiveIntake {
    public static CachedMotor motor;
    public static ServoPlus Blocker;
    public static double Block = 105, UnBlock = 245;
    synchronized public static void powerOn(double s){
        double t = System.currentTimeMillis();
        new Thread(() -> {
            while(System.currentTimeMillis() - t < s / 1000);
            powerOn();
        });
    }
    synchronized public static void powerOff(double s){
        double t = System.currentTimeMillis();
        new Thread(() -> {
            while (System.currentTimeMillis() - t < s / 1000) ;
            powerOff();
        });
    }

    synchronized public static void powerOn(){
        motor.setPower(-1);
    }
    synchronized public static void powerOff(){
        motor.setPower(0);
    }
    synchronized public static void Reverse(){
        motor.setPower(1);
    }
    synchronized public static void BlockIntake(){ Blocker.setAngle(Block); }
    synchronized public static boolean isBlocked(){ return Blocker.getAngle() == Block; }
    synchronized public static void UnblockIntake(){
        if(Blocker == null) return;
        Blocker.setAngle(UnBlock);
    }
}
