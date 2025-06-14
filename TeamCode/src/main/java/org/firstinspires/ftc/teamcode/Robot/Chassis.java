package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;


@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static double FLd = 1, FRd = -1, BLd = 1, BRd = -1;
    public static double SplineDoneNess = 0;
    public static boolean PuttingSpecimens = false;
    public static boolean Autonomous = false;
    public static boolean DoingSpecimens = false;

    public static void drive(double x, double y, double r){
        if(PuttingSpecimens) {
            r *= 0.7;
            x *= 0.8;
            y *= 0.8;
        }
        Robot.telemetry.addData("FL PC", FL.getCurrent(CurrentUnit.AMPS));
        Robot.telemetry.addData("FR PC", FR.getCurrent(CurrentUnit.AMPS));
        Robot.telemetry.addData("BL PC", BL.getCurrent(CurrentUnit.AMPS));
        Robot.telemetry.addData("BR PC", BR.getCurrent(CurrentUnit.AMPS));
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
        double fl, bl, fr, br;

        fl = (y + x + r) / d;
        bl = (y - x + r) / d;
        fr = (y - x - r) / d;
        br = (y + x - r) / d;

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
    public static PIDController Strafe = new PIDController(0.01, 0.0, 0.001),
                                Forward = new PIDController(-0.012, -0.02, -0.002),
                                Heading = new PIDController(0.85, 0.5, 0.07);
    public static PIDCoefficients FullExtendoHeading = new PIDCoefficients(0.3,0.0015,0.06);
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

        Strafe.kS = 0.0;
        Forward.kS = -0.04;
        Heading.kS = 0.02;

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
    public static void setHeadingProfiles(double a, double d, double mv){
        hProfile = new AsymmetricMotionProfile(mv, a, d);
    }
    public static boolean asyncFollow = false;
    public static boolean linearHeading = false;
    private static LinearFunction headingInterpolation;
    private static double totalDistanceToTravel = 0;
    private static int point = 0;

    public static void profiledFollow(SparkFunOTOS.Pose2D pose){
        RobotLog.dd("targetPosition", pose.x + ", " + pose.y + ", " + Math.toRadians(pose.h));
        pose.h = Localizer.normalizeRadians(pose.h);
//        pointsToFollow.clear();
//        pointsToFollow.add(pose);
        xProfile.startMotion(Localizer.getCurrentPosition().x, pose.x, point == 0 ? xProfile.acceleration : 1e10, point == pointsToFollow.size() ? xProfile.deceleration : 1e10);
        yProfile.startMotion(Localizer.getCurrentPosition().y, pose.y, point == 0 ? yProfile.acceleration : 1e10, point == pointsToFollow.size() ? yProfile.deceleration : 1e10);
        hProfile.startMotion(Localizer.getCurrentPosition().h, pose.h, point == 0 ? hProfile.acceleration : 1e10, point == pointsToFollow.size() ? hProfile.deceleration : 1e10);
        /*if(point == 0 && point == pointsToFollow.size()){
            xProfile.changeInitialAndEndVelocity(0, 0);
            yProfile.changeInitialAndEndVelocity(0, 0);
            hProfile.changeInitialAndEndVelocity(0, 0);
        }
        if(point == 0) {
//            Robot.telemetry.addLine("STARTPOINT");
            xProfile.changeInitialAndEndVelocity(0, xProfile.maxVelocity / 2);
            yProfile.changeInitialAndEndVelocity(0, yProfile.maxVelocity / 2);
            hProfile.changeInitialAndEndVelocity(0, hProfile.maxVelocity / 2);
        } else if(point == pointsToFollow.size()){
//            Robot.telemetry.addLine("ENDPOINT");
            xProfile.changeInitialAndEndVelocity(xProfile.maxVelocity / 2, 0);
            yProfile.changeInitialAndEndVelocity(yProfile.maxVelocity / 2, 0);
            hProfile.changeInitialAndEndVelocity(hProfile.maxVelocity / 2, 0);
        } else {
//            Robot.telemetry.addLine("INTERPOINT");
            xProfile.changeInitialAndEndVelocity(xProfile.maxVelocity / 2, xProfile.maxVelocity / 2);
            yProfile.changeInitialAndEndVelocity(yProfile.maxVelocity / 2, yProfile.maxVelocity / 2);
            hProfile.changeInitialAndEndVelocity(hProfile.maxVelocity / 2, hProfile.maxVelocity / 2);
        }*/
//        xProfile.startMotion(Localizer.getCurrentPosition().x, pose.x);
//        yProfile.startMotion(Localizer.getCurrentPosition().y, pose.y);
//        hProfile.startMotion(Localizer.getCurrentPosition().h, pose.h);

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
        profiledCurve(new ArrayList<SparkFunOTOS.Pose2D>(Collections.singletonList(pose)));
        linearHeading = true;
    }
    public static void profiledCurve(List<SparkFunOTOS.Pose2D> points){
        SplineDoneNess = 0;
        pointsToFollow = points;
        point = 0;
//        profiledFollow(points.get(0));
        asyncFollow = true;
    }
    public static void setHeading(double h){
        setTargetPosition(new SparkFunOTOS.Pose2D(targetPosition.x, targetPosition.y, h));
    }

    public static void Update(){
        try {
            Robot.telemetry.addLine(point + " / " + pointsToFollow.size() + " of motion done");
            if (asyncFollow) {
                xProfile.update();
                yProfile.update();
                hProfile.update();
                double h = hProfile.getPosition();
                if (linearHeading) {
                    double soFar = Math.sqrt((xProfile.getPosition() - xProfile.getTargetPosition()) * (xProfile.getPosition() - xProfile.getTargetPosition()) +
                            (yProfile.getPosition() - yProfile.getTargetPosition()) * (yProfile.getPosition() - yProfile.getTargetPosition()));
//                Robot.telemetry.addData("soFar", soFar);
                    h = headingInterpolation.getOutput((totalDistanceToTravel - soFar) / totalDistanceToTravel);
                }
                setTargetPosition(new SparkFunOTOS.Pose2D(xProfile.getPosition(), yProfile.getPosition(), h));
//            Robot.telemetry.addData("x precent", xProfile.getPrecentOfMotion());
//            Robot.telemetry.addData("y precent", yProfile.getPrecentOfMotion());
                if (Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), new SparkFunOTOS.Pose2D(xProfile.getTargetPosition(), yProfile.getTargetPosition(), 0)) < 200 || point == 0) {
                    if (point >= pointsToFollow.size()) {
                        if (xProfile.motionEnded() && yProfile.motionEnded() && (linearHeading || hProfile.motionEnded())) {
                            asyncFollow = false;
                        }
                    } else {
//                    Robot.telemetry.addLine("FOLLOW NEXT");
                        SplineDoneNess = point * (100.0 / pointsToFollow.size());
                        profiledFollow(pointsToFollow.get(point++));
                    }
                }
            }
        } catch (Exception e){
            RobotLog.ee("Error during auton", e.getMessage());
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
        if(Robot.VOLTAGE > 12.8){
            p *= 12.8 / Robot.VOLTAGE;
        }
        drive(yP * p, -xP * p, hP * p);
    }
    public static double getPrecentageOfMotionDone(){

        final double maxContributedProcentage = 100.0 / pointsToFollow.size();

        if(point < pointsToFollow.size() - 1) return 0;
        double soFar = Math.sqrt((xProfile.getPosition() - xProfile.getTargetPosition()) * (xProfile.getPosition() - xProfile.getTargetPosition()) +
                (yProfile.getPosition() - yProfile.getTargetPosition()) * (yProfile.getPosition() - yProfile.getTargetPosition()));

        if(linearHeading){
            soFar = Math.sqrt((Localizer.getCurrentPosition().x - xProfile.getTargetPosition()) * (Localizer.getCurrentPosition().x - xProfile.getTargetPosition()) +
                    (Localizer.getCurrentPosition().y - yProfile.getTargetPosition()) * (Localizer.getCurrentPosition().y - yProfile.getTargetPosition()));
        }
//        double totalDistance

        return SplineDoneNess + (1 - soFar / totalDistanceToTravel) * maxContributedProcentage;
//        return 0;
    }
}
