package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
public class Chassis {
    public static PIDController xController = new PIDController(0, 0, 0), yController = new PIDController(0, 0, 0);
    public static PIDController hController = new PIDController(0, 0, 0);
    public static double ReachedTargetPositionTreshHold = 1; // tollerance
    public static StandardTrackingWheelLocalizer localizer;

    private static Pose2d targetPosition;


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

    public static void SetTargetPosition(Pose2d pos){
        targetPosition = pos;
        hController.setTargetPosition(pos.getHeading());
        xController.setTargetPosition(pos.getX());
        yController.setTargetPosition(pos.getY());
    }
    public static Pose2d GetCurrentPosition(){
        return localizer.getPoseEstimate();
    }
    public static void Update(){
        drive(xController.calculatePower(GetCurrentPosition().getX()), yController.calculatePower(GetCurrentPosition().getY()), hController.calculatePower(GetCurrentPosition().getHeading()));
    }

}
