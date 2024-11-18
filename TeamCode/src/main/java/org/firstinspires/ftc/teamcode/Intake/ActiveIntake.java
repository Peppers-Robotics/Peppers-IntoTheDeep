package org.firstinspires.ftc.teamcode.Intake;

import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

public class ActiveIntake {
    public static CachedMotor motor;
    public static ServoPlus Blocker;
    public static double Block = 1, UnBlock = 0;

    synchronized public static void powerOn(){
        motor.setPower(1);
    }
    synchronized public static void powerOff(){
        motor.setPower(0);
    }
    synchronized public static void Reverse(){
        motor.setPower(-1);
    }
    synchronized public static void BlockIntake(){ Blocker.setAngle(Block); }
    synchronized public static boolean isBlocked(){ return Blocker.getAngle() == Block; }
    synchronized public static void UnblockIntake(){ Blocker.setAngle(UnBlock); }
}
