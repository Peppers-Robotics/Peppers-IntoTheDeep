package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
public class Chassis {
    public static PIDController xController = new PIDController(-0.15, -0.001, -0.014), yController = new PIDController(-0.15, -0.001, 0.015);
    public static PIDController hController = new PIDController(1, 0, -0.04);
    public static Localizer localizer;
    private static Pose2D targetPosition;
    public static IMU imu;


    public static CachedMotor FL, FR, BL, BR;
    public static double rFL = -1, rFR = 1, rBL = -1, rBR = 1;
    public static void drive(double x, double y, double h){
        double pow = 0.5;
        if(Controls.gamepad1 != null) {
            if (Controls.gamepad1.gamepad.cross) pow = 1;
            h = h * 0.7;
        } else {
            pow = 1;
        }

        FL.setPower((y + x + h) / rFL * pow);
        FR.setPower((y - x - h) / rFR * pow);
        if(rBL != 0)
            BL.setPower((y - x + h) / rBL * pow);
        if(rBL != 0)
            BR.setPower((y + x - h) / rBR * pow);
    }

    public static void SetTargetPosition(Pose2D pos){
        xController.setTargetPosition(0, targetPosition != pos);
        yController.setTargetPosition(0, targetPosition != pos);
        hController.setTargetPosition(0, targetPosition != pos);
        targetPosition = pos;
    }
    public static Pose2D GetCurrentPosition(){
        return new Pose2D(localizer.getPoseEstimate());
    }
    public static Pose2D GetTargetPosition(){
        return targetPosition;
    }
    public static double DistanceToTarget(){
        return Math.sqrt(
            (GetCurrentPosition().x - targetPosition.x) * (GetCurrentPosition().x - targetPosition.x) +
            (GetCurrentPosition().y - targetPosition.y) * (GetCurrentPosition().y - targetPosition.y)
        );
    }
    public static void Update(){
        localizer.update();
        Pose2D normal = new Pose2D(GetTargetPosition().x - GetCurrentPosition().x, GetTargetPosition().y - GetCurrentPosition().y, GetCurrentPosition().h);
        double He = normal.h - targetPosition.h;
        while(He < -Math.PI) He += 2*Math.PI;
        while(He > Math.PI) He -= 2 * Math.PI;
        Pose2D error = new Pose2D(
                Math.cos(GetCurrentPosition().h) * normal.x + Math.sin(GetCurrentPosition().h) * normal.y,
                Math.sin(GetCurrentPosition().h) * normal.x - Math.cos(GetCurrentPosition().h) * normal.y,
                He
        );
        double xP = xController.calculatePower(error.x, localizer.getPoseVelocity().getX()),
                yP = yController.calculatePower(error.y, localizer.getPoseVelocity().getY()),
                hP = hController.calculatePower(error.h, localizer.getPoseVelocity().getHeading());
        drive(yP, xP, hP);
    }
    public static double getYaw(AngleUnit u) {
        return imu.getRobotYawPitchRollAngles().getYaw(u);
    }

}
