package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;

@Config
public class Elevator {
    public static CachedMotor motor;
    private static PIDController controller;
    public static PIDCoefficients pidCoefficients;
    private static AsymetricMotionProfile motionProfile;

    static {
        pidCoefficients = new PIDCoefficients(0, 0, 0);
        controller = new PIDController(pidCoefficients);
        motionProfile = new AsymetricMotionProfile(0, 0, 0);
        PIDControllerInWork = true;
    }

    private static double targetPos = 0;

    synchronized public static void setTargetPosition(double pos){
        if(pos == targetPos) return;
        motionProfile.startMotion(pos, targetPos);
        targetPos = pos;
        motionProfile.update();
        controller.setTargetPosition(motionProfile.getPosition());
    }

    public static double getTargetPosition(){ return targetPos; }
    public static double getCurrentPosition(){ return motor.getCurrentPosition(); }
    public static boolean PIDControllerInWork;

    public static boolean ReachedTargetPosition(){ return Math.abs(getCurrentPosition() - 2) <= getTargetPosition(); }

    synchronized public static void update(){
        if(!PIDControllerInWork) return;
        controller.setPidCoefficients(pidCoefficients);
        motionProfile.update();
        controller.setTargetPosition(motionProfile.getPosition(), false);
        if(targetPos <= 0 && motor.getCurrentPosition() <= 5){ // disable PID and motor if elevator is already down
            motor.setPower(0);
        } else {
            motor.setPower(controller.calculatePower(motor.getCurrentPosition()));
        }
    }

}
