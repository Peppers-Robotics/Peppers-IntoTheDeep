package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

@Config
public class Chassis {
    public static PIDController transitionalPID = new PIDController(0, 0, 0);
    public static PIDController headingPID = new PIDController(0, 0, 0);
    public static double ReachedTargetPositionTreshHold = 50; // in mm

    private static Pose2D targetPosition;


    public static CachedMotor FL, FR, BL, BR;
    public static double rFL = -1, rFR = 1, rBL = -1, rBR = 1;
    public static void drive(double x, double y, double h){
        h = h * 0.7;
        double pow = 0.5;
        if(Controls.gamepad1.gamepad.cross) pow = 1;

        FL.setPower((y + x + h) / rFL * pow);
        FR.setPower((y - x - h) / rFR * pow);
        if(rBL != 0)
            BL.setPower((y - x + h) / rBL * pow);
        if(rBL != 0)
            BR.setPower((y + x - h) / rBR * pow);
    }

}
