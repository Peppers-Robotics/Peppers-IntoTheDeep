package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;

@Config
public class DropDown {
    public static ServoPlus servo;
    public static double Up = 0;
    public static double retractedDown = 130, extendedDown = 120;

    public static void setDown(double precent){
        double down = retractedDown + (extendedDown - retractedDown) / (Extendo.getMaxPosition()) * Extendo.getCurrentPosition();
        double angle = Up + (Up - down) * precent;
        servo.setAngle(angle);
    }
}
