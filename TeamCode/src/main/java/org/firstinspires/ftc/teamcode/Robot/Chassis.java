package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

import java.nio.channels.FileChannel;

@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static void drive(double x, double y, double r){
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
        double fl = (y + x + r) / d;
        double bl = (y - x + r) / d;
        double fr = (y - x - r) / d;
        double br = (y + x - r) / d;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }
    public static void driveFiledCentric(double x, double y, double r){
        double xP = x * Math.cos(-Localizer.getCurrentPosition().getHeading(AngleUnit.RADIANS)) - y * Math.sin(-Localizer.getCurrentPosition().getHeading(AngleUnit.RADIANS));
        double yP = y * Math.cos(-Localizer.getCurrentPosition().getHeading(AngleUnit.RADIANS)) + x * Math.sin(-Localizer.getCurrentPosition().getHeading(AngleUnit.RADIANS));
        drive(xP, yP, r);
    }

    // Autonomous implementation

    private static Pose2D targetPosition;
    public static PIDController Translational = new PIDController(0, 0, 0),
                                Heading       = new PIDController(0, 0, 0);

    public static void setTargetPosition(Pose2D pose){
        targetPosition = pose;
    }
    private static double getDistance(Pose2D A, Pose2D B){
        return Math.sqrt(
                (A.getX(DistanceUnit.MM) - B.getX(DistanceUnit.MM)) * (A.getX(DistanceUnit.MM) - B.getX(DistanceUnit.MM)) +
                (A.getY(DistanceUnit.MM) - B.getY(DistanceUnit.MM)) * (A.getY(DistanceUnit.MM) - B.getY(DistanceUnit.MM)) );
    }
    public static void Update(){
        double error = getDistance(Localizer.getCurrentPosition(), targetPosition);
        double directionalVelocity = Math.sqrt(Localizer.getVelocity().getX(DistanceUnit.MM) * Localizer.getVelocity().getX(DistanceUnit.MM) +
                                     Localizer.getVelocity().getY(DistanceUnit.MM) * Localizer.getVelocity().getY(DistanceUnit.MM));
        double pow = Translational.calculatePower(error, directionalVelocity);

        double Herror = Localizer.getCurrentPosition().getHeading(AngleUnit.DEGREES) - targetPosition.getHeading(AngleUnit.DEGREES);
        double hP = Heading.calculatePower(Herror, Localizer.getVelocity().getHeading(AngleUnit.DEGREES));
        double xP = pow * Math.sin(Herror);
        double yP = pow * Math.cos(Herror);

        drive(xP, yP, hP);
    }
}
