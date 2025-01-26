package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class DropDown {
    public static ServoPlus servo;
    public static double Up = 270;
    public static double retractedDown = 180, extendedDown = 205;

    public static void setDown(double precent){
        double down = retractedDown + (extendedDown - retractedDown) / (Extendo.getMaxPosition()) * Extendo.getCurrentPosition();

        double angle = Up - (Up - down) * precent;
        servo.setAngle(angle);
        Robot.telemetry.addData("Extendo Pos", Extendo.getCurrentPosition());
    }
}