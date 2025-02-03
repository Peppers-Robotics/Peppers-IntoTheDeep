package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;

import java.util.List;


@Config
public class Chassis {
    public static CachedMotor FL, FR, BL, BR;
    public static int FLd = 1, FRd = -1, BLd = 1, BRd = -1;
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

    // Autonomous implementation



    private static SparkFunOTOS.Pose2D targetPosition = new SparkFunOTOS.Pose2D();
    public static double angleStrafe = 0, angleForward = 10;
    public static PIDController Strafe = new PIDController(0.002, 0.001, 0.01),
                                Forward = new PIDController(-0.007, -0.005, 0.03),
                                Heading       = new PIDController(2, 0, -0.1);

    public static void setTargetPosition(SparkFunOTOS.Pose2D pose){
        Strafe.setTargetPosition(0);
        Forward.setTargetPosition(0);
        Heading.setTargetPosition(0);
        targetPosition = pose;
    }
    static{
        Strafe.setFreq(500);
        Forward.setFreq(500);
        Heading.setFreq(500);
    }

    public static SparkFunOTOS.Pose2D getTargetPosition(){
        return targetPosition;
    }


    public static double xAccel = 4000, yAccel = 4000, xMV = 3000, yMV = 3000, xDecc = 2500, yDecc = 1000;
    public static AsymmetricMotionProfile xProfile = new AsymmetricMotionProfile(3000, 4000, 2500),
            yProfile = new AsymmetricMotionProfile(3000, 4000, 1000),
            hProfile = new AsymmetricMotionProfile(Math.PI * 2, Math.PI, Math.PI);
    public static void resetProfiles(){
        xProfile = new AsymmetricMotionProfile(xMV, xAccel, xDecc);
        yProfile = new AsymmetricMotionProfile(yMV, yAccel, yDecc);
    }
    public static void setProfiles(double xA, double yA, double xMV, double yMV, double xD, double yD){
        xProfile = new AsymmetricMotionProfile(xMV, xA, xD);
        yProfile = new AsymmetricMotionProfile(yMV, yA, yD);
    }
    public static boolean asyncFollow = false;
    public static boolean linearHeading = false;
    private static LinearFunction headingInterpolation;
    private static double totalDistanceToTravel = 0;

    public static void profiledFollow(SparkFunOTOS.Pose2D pose){
        xProfile.startMotion(Localizer.getCurrentPosition().x, pose.x);
        yProfile.startMotion(Localizer.getCurrentPosition().y, pose.y);
        hProfile.startMotion(Localizer.getCurrentPosition().h, pose.h);
        totalDistanceToTravel = Math.sqrt((Localizer.getCurrentPosition().x - pose.x) * (Localizer.getCurrentPosition().x - pose.x) +
                                          (Localizer.getCurrentPosition().y - pose.y) * (Localizer.getCurrentPosition().y - pose.y));
        asyncFollow = true;
        linearHeading = false;
    }
    public static void profiledLinearHeading(SparkFunOTOS.Pose2D pose){
        headingInterpolation = new LinearFunction(Localizer.getCurrentPosition().h, pose.h);
        profiledFollow(pose);
        linearHeading = true;
    }
    public static void profiledBeziereCurve(List<SparkFunOTOS.Pose2D> points){

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
            if(xProfile.motionEnded() && yProfile.motionEnded() && (linearHeading || hProfile.motionEnded())) asyncFollow = false;
        }


        SparkFunOTOS.Pose2D normal = new SparkFunOTOS.Pose2D(
                getTargetPosition().x - Localizer.getCurrentPosition().x,
                getTargetPosition().y - Localizer.getCurrentPosition().y,
                getTargetPosition().h - Localizer.getCurrentPosition().h);

        double Herror = normal.h;
        while(Herror < -Math.PI) Herror += Math.PI * 2;
        while(Herror >  Math.PI) Herror -= Math.PI * 2;
        SparkFunOTOS.Pose2D error = new SparkFunOTOS.Pose2D(
                Math.cos(Localizer.getCurrentPosition().h) * normal.x + Math.sin(Localizer.getCurrentPosition().h) * normal.y,
                Math.sin(Localizer.getCurrentPosition().h) * normal.x - Math.cos(Localizer.getCurrentPosition().h) * normal.y,
                Herror
        );

        double xP = Strafe.calculatePower(error.x, Localizer.getVelocity().x);
        double yP = Forward.calculatePower(error.y, Localizer.getVelocity().y);
        double hP = Heading.calculatePower(error.h, Localizer.getVelocity().h);
        Robot.telemetry.addData("xError", error.x);
        Robot.telemetry.addData("yError", error.y);
        Robot.telemetry.addData("hError", error.h);

        double p = 1;
        if(Robot.VOLTAGE > 13){
            p = 13.f / Robot.VOLTAGE;
        }

        drive(yP * p, -xP * p, hP * p);
    }
    public static double getPrecentageOfMotionDone(){
        double soFar = Math.sqrt((xProfile.getPosition() - xProfile.getTargetPosition()) * (xProfile.getPosition() - xProfile.getTargetPosition()) +
                (yProfile.getPosition() - yProfile.getTargetPosition()) * (yProfile.getPosition() - yProfile.getTargetPosition()));
        return (1 - soFar / totalDistanceToTravel) * 100.f;
    }
}
