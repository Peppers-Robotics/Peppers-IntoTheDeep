package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;


@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static double FLd = 1, FRd = -1, BLd = 1, BRd = -1;
    public static void drive(double x, double y, double r){
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
        double fl = (y + x + r) / d;
        double bl = (y - x + r) / d;
        double fr = (y - x - r) / d;
        double br = (y + x - r) / d;

        FL.setPower(fl * FLd);
        FR.setPower(fr * FRd);
        BL.setPower(bl * BLd);
        BR.setPower(br * BRd);
    }
    public static double holdHeading(){
        double err = getTargetPosition().h - Localizer.getCurrentPosition().h;
        return Heading.calculatePower(err, Localizer.getVelocity().h);
    }


    public static void drivePolar(double mod, double alpha){

    }

    // Autonomous implementation

    private static SparkFunOTOS.Pose2D targetPosition = new SparkFunOTOS.Pose2D();
    public static PIDController Strafe = new PIDController(0.01, 0.05, 0.002),
                                Forward = new PIDController(-0.012, -0.02, -0.002),
                                Heading       = new PIDController(0.85, 0.5, 0.07);
    private static List<SparkFunOTOS.Pose2D> pointsToFollow;

    public static void setTargetPosition(SparkFunOTOS.Pose2D pose){
        Strafe.setTargetPosition(0);
        Forward.setTargetPosition(0);
        pose.h = Localizer.normalizeRadians(pose.h);
        Heading.setTargetPosition(0);
        targetPosition = pose;
    }
    static{
        Strafe.setFreq(30);
        Forward.setFreq(30);
        Heading.setFreq(30);
    }

    public static SparkFunOTOS.Pose2D getTargetPosition(){
        return targetPosition;
    }
    public static void holdOrientation(double x, double y, double h){
        double e = h - Localizer.getCurrentPosition().h;
        e = Localizer.normalizeRadians(e);
        double rot = Heading.calculatePower(e, Localizer.getVelocity().h);
        drive(x, y, rot);
    }

    public static double xAccel = 4000, yAccel = 4000, xMV = 3000, yMV = 3000, xDecc = 2500, yDecc = 1000;
    public static AsymmetricMotionProfile xProfile = new AsymmetricMotionProfile(4000, 3000, 1000),
            yProfile = new AsymmetricMotionProfile(4000, 3000, 1000),
            hProfile = new AsymmetricMotionProfile(Math.PI * 6, Math.PI * 4, Math.PI * 3);
    public static void resetProfiles(){
        xProfile.maxVelocity = xMV;
        xProfile.acceleration = xAccel;
        xProfile.deceleration = xDecc;
        xProfile = new AsymmetricMotionProfile(xMV, xAccel, xDecc);
        xProfile.startMotion(xProfile.getPosition(), xProfile.getTargetPosition(), xProfile.getVelocity());

        yProfile.maxVelocity = yMV;
        yProfile.acceleration = yAccel;
        yProfile.deceleration = yDecc;
        yProfile = new AsymmetricMotionProfile(yMV, yAccel, yDecc);

        yProfile.startMotion(yProfile.getPosition(), yProfile.getTargetPosition(), yProfile.getVelocity());
    }
    public static void setProfiles(double xA, double yA, double xMV, double yMV, double xD, double yD){
        xProfile = new AsymmetricMotionProfile(xMV, xA, xD);
        yProfile = new AsymmetricMotionProfile(yMV, yA, yD);
    }
    public static boolean asyncFollow = false;
    public static boolean linearHeading = false;
    private static LinearFunction headingInterpolation;
    private static double totalDistanceToTravel = 0;
    private static int point = 0;

    public static void profiledFollow(SparkFunOTOS.Pose2D pose){
        pose.h = Localizer.normalizeRadians(pose.h);
        pointsToFollow.clear();
        pointsToFollow.add(pose);
        xProfile.startMotion(Localizer.getCurrentPosition().x, pose.x, point == 0 ? 0 : xProfile.maxVelocity, point == pointsToFollow.size() - 1 ? 0 : xProfile.maxVelocity);
        yProfile.startMotion(Localizer.getCurrentPosition().y, pose.y, point == 0 ? 0 : yProfile.maxVelocity, point == pointsToFollow.size() - 1 ? 0 : yProfile.maxVelocity);
        hProfile.startMotion(Localizer.getCurrentPosition().h, pose.h, point == 0 ? 0 : hProfile.maxVelocity, point == pointsToFollow.size() - 1 ? 0 : hProfile.maxVelocity);
        totalDistanceToTravel = Math.sqrt((Localizer.getCurrentPosition().x - pose.x) * (Localizer.getCurrentPosition().x - pose.x) +
                                          (Localizer.getCurrentPosition().y - pose.y) * (Localizer.getCurrentPosition().y - pose.y));
        linearHeading = false;
    }
    public static void unprofiledLinearHeading(SparkFunOTOS.Pose2D pose){
        profiledFollow(pose);
        pose.h = Localizer.normalizeRadians(pose.h);
        xProfile.setInstant(pose.x);
        yProfile.setInstant(pose.y);
        hProfile.setInstant(pose.h);
        headingInterpolation = new LinearFunction(Localizer.getCurrentPosition().h, pose.h);
        linearHeading = true;
    }
    public static void profiledLinearHeading(SparkFunOTOS.Pose2D pose){
        pose.h = Localizer.normalizeRadians(pose.h);
        headingInterpolation = new LinearFunction(Localizer.getCurrentPosition().h, pose.h);
//        profiledFollow(pose);
        profiledCurve(new ArrayList<SparkFunOTOS.Pose2D>((Collection) pose));
        linearHeading = true;
    }
    public static void profiledCurve(List<SparkFunOTOS.Pose2D> points){
        pointsToFollow = points;
        point = 0;
        asyncFollow = true;
    }
    public static void setHeading(double h){
        setTargetPosition(new SparkFunOTOS.Pose2D(targetPosition.x, targetPosition.y, h));
    }

    public static void Update(){
        if(asyncFollow){
            xProfile.update();
            yProfile.update();
            hProfile.update();
            double h = hProfile.getPosition();
            if(linearHeading){
                double soFar = Math.sqrt((xProfile.getPosition() - xProfile.getTargetPosition()) * (xProfile.getPosition() - xProfile.getTargetPosition()) +
                                (yProfile.getPosition() - yProfile.getTargetPosition()) * (yProfile.getPosition() - yProfile.getTargetPosition()));
                Robot.telemetry.addData("soFar", soFar);
                h = headingInterpolation.getOutput((totalDistanceToTravel - soFar) / totalDistanceToTravel);
            }
            setTargetPosition(new SparkFunOTOS.Pose2D(xProfile.getPosition(), yProfile.getPosition(), h));
            if(xProfile.motionEnded() && yProfile.motionEnded() && (linearHeading || hProfile.motionEnded())){
                if(point >= pointsToFollow.size() - 1) asyncFollow = false;
                else {
                    profiledFollow(pointsToFollow.get(++point));
                }
            }
        }

        SparkFunOTOS.Pose2D normal = new SparkFunOTOS.Pose2D(
                getTargetPosition().x - Localizer.getCurrentPosition().x,
                getTargetPosition().y - Localizer.getCurrentPosition().y,
                getTargetPosition().h - Localizer.getCurrentPosition().h);

        double Herror = Localizer.normalizeRadians(normal.h);
        double h = Localizer.getCurrentPosition().h;
        while(h < 0) h += Math.PI * 2;
        while(h > 2*Math.PI) h -= Math.PI * 2;
        SparkFunOTOS.Pose2D error = new SparkFunOTOS.Pose2D(
                Math.cos(h) * normal.x + Math.sin(h) * normal.y,
                Math.sin(h) * normal.x - Math.cos(h) * normal.y,
                Herror
        );

        double xP = Strafe.calculatePower(error.x);
        double yP = Forward.calculatePower(error.y);
        double hP = Heading.calculatePower(error.h);

        Robot.telemetry.addData("xError", error.x);
        Robot.telemetry.addData("yError", error.y);
        Robot.telemetry.addData("hError", Math.toDegrees(error.h));
        double p = 1;
        if(Robot.VOLTAGE > 13.2){
            p *= 13.2 / Robot.VOLTAGE;
        }
        drive(yP * p, -xP * p, hP * p);
    }
    public static double getPrecentageOfMotionDone(){
        double soFar = Math.sqrt((xProfile.getPosition() - xProfile.getTargetPosition()) * (xProfile.getPosition() - xProfile.getTargetPosition()) +
                (yProfile.getPosition() - yProfile.getTargetPosition()) * (yProfile.getPosition() - yProfile.getTargetPosition()));

        if(linearHeading){
            soFar = Math.sqrt((Localizer.getCurrentPosition().x - xProfile.getTargetPosition()) * (Localizer.getCurrentPosition().x - xProfile.getTargetPosition()) +
                    (Localizer.getCurrentPosition().y - yProfile.getTargetPosition()) * (Localizer.getCurrentPosition().y - yProfile.getTargetPosition()));
        }

        return (1 - soFar / totalDistanceToTravel) * 100.f;
    }
}
