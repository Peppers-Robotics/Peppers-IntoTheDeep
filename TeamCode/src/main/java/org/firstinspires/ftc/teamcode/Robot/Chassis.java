package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;


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
    public static PIDController Strafe = new PIDController(0.005, 0, 0.04),
                                Forward = new PIDController(-0.005, 0, -0.025),
                                Heading       = new PIDController(1.5, 0, -0.001);

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

    public static void Update(){

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
        drive(yP, xP, hP);
    }
}
