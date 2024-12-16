package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;

@Config
public class ArmMK2 {
    public static ServoPlus ArmL, ArmR, Turret;
    public static AsymmetricMotionProfile armProfile = new AsymmetricMotionProfile(4e3, 7e3, 7e4),
                                          turretProfile = new AsymmetricMotionProfile(6e3, 6e3, 5e4);

    public static void setArmAngle(double angle){
        armProfile.startMotion(armProfile.getTargetPosition(), angle);
    }
    public static void setTurretAngle(double angle){
        turretProfile.startMotion(turretProfile.getTargetPosition(), angle);
    }
    public static void update(){


        ArmR.setAngle(-armProfile.getPosition()); // they are mirrored?
        ArmL.setAngle(armProfile.getPosition());
        Turret.setAngle(turretProfile.getPosition());
        armProfile.update();
        turretProfile.update();
    }
    public static boolean motionCompleted(){
        return ArmMotionCompleted() && TurretMotionCompleted();
    }
    public static double getCurrentArmAngle(){
        return armProfile.getPosition();
    }
    public static double getCurrentTurretAngle(){
        return turretProfile.getPosition();
    }
    public static boolean ArmMotionCompleted(){
        return armProfile.motionEnded();
    }
    public static boolean TurretMotionCompleted(){
        return turretProfile.motionEnded();
    }
}
