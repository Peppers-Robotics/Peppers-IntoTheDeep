package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;

public class LineTo extends Task {
    public AsymmetricMotionProfile xProfile = new AsymmetricMotionProfile(1270, 1000, 2500),
            yProfile = new AsymmetricMotionProfile(1270, 1000, 2500),
            hProfile = new AsymmetricMotionProfile(Math.PI * 2, Math.PI, Math.PI);
    public LineTo(SparkFunOTOS.Pose2D pose){
        xProfile.startMotion(Localizer.getCurrentPosition().x, pose.x);
        yProfile.startMotion(Localizer.getCurrentPosition().y, pose.y);
        hProfile.startMotion(Localizer.getCurrentPosition().h, pose.h);
    }

    @Override
    public boolean Run() {
        xProfile.update();
        yProfile.update();
        hProfile.update();

        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(xProfile.getPosition(), yProfile.getPosition(), hProfile.getPosition()));

        return xProfile.motionEnded() && yProfile.motionEnded() && hProfile.motionEnded() && Localizer.getDistanceFromTwoPoints(Chassis.getTargetPosition(), Localizer.getCurrentPosition()) < 50;
    }
}
