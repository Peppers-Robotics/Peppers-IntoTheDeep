package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.TunablePose2d;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelLocalizer;

@Config
public class Chassis {
    public static ThreeDeadWheelLocalizer localizer;

    public static PIDController transitionalPID = new PIDController(0, 0, 0);
    public static PIDController headingPID = new PIDController(0, 0, 0);
    public static double ReachedTargetPositionTreshHold = 50; // in mm

    private static Pose2D targetPosition;


    public static CachedMotor FL, FR, BL, BR;
    public static double rFL = -1, rFR = 1, rBL = -1, rBR = 1;
    public static void drive(double x, double y, double h){
        double div = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(h), 1);
        double pow = 1;
        if(Controls.gamepad1.gamepad.cross) pow = 0.7;

        FL.setPower((y + x + h) / div * rFL * pow);
        FR.setPower((y - x - h) / div * rFR * pow);
        BL.setPower((y - x + h) / div * rBL * pow);
        BR.setPower((y + x - h) / div * rBR * pow);
    }

    public static void InitializeTracking(HardwareMap hm){
        localizer = new ThreeDeadWheelLocalizer(hm);
    }
    public static void setTargetPosition(Pose2D pos){
        targetPosition = pos;
        transitionalPID.setTargetPosition(0);
        headingPID.setTargetPosition(0);
    }
    public static void setTargetPosition(TunablePose2d pos){
        setTargetPosition(new Pose2D(DistanceUnit.MM, pos.x, pos.y, AngleUnit.RADIANS, pos.h));
    }
    public static boolean ReachedTargetPosition(){
        return ThreeDeadWheelLocalizer.getDistance(targetPosition, localizer.getCurrentPosition()) <= ReachedTargetPositionTreshHold;
    }
    public static boolean ReachedTargetPosition(double treshhold){
        return ThreeDeadWheelLocalizer.getDistance(targetPosition, localizer.getCurrentPosition()) <= treshhold;
    }
    public static double getDistanceToTarget(TunablePose2d pos){
        return getDistanceToTarget(new Pose2D(DistanceUnit.MM, pos.x, pos.y, AngleUnit.RADIANS, pos.h));
    }
    public static double getDistanceToTarget(Pose2D pos){
        return ThreeDeadWheelLocalizer.getDistance(targetPosition, localizer.getCurrentPosition());
    }

    public static void Update(){
        double distanceToTarget = ThreeDeadWheelLocalizer.getDistance(targetPosition, localizer.getCurrentPosition());
        double tP = transitionalPID.calculatePower(distanceToTarget);
        double hP = headingPID.calculatePower(distanceToTarget);

        drive((targetPosition.getX(DistanceUnit.MM) - localizer.getCurrentPosition().getX(DistanceUnit.MM)) / distanceToTarget,
                (targetPosition.getY(DistanceUnit.MM) - localizer.getCurrentPosition().getY(DistanceUnit.MM)) / distanceToTarget,
                hP);

        localizer.update();
    }
}
