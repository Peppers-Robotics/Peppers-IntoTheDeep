package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;

@Config
public class DropDown {
    public static ServoPlus left;
    public static double IdleUp = 0, UpIntake = 10, MiddleIntake = 90, LowIntake = 180;
    public static AsymmetricMotionProfile DropDownProfile = new AsymmetricMotionProfile(0, 0, 0);

    public static void GoUp(){
        DropDownProfile.startMotion(DropDownProfile.getPosition(), IdleUp);
    }
    public static void IntakeUp(){
        DropDownProfile.startMotion(DropDownProfile.getPosition(), UpIntake);
    }
    public static void IntakeMiddle(){
        DropDownProfile.startMotion(DropDownProfile.getPosition(), MiddleIntake);
    }
    public static void IntakeLow(){
        DropDownProfile.startMotion(DropDownProfile.getPosition(), LowIntake);
    }

    public static void Update(){
        DropDownProfile.update();
        left.setAngle(DropDownProfile.getPosition());
    }
}
