package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class PinPointLocalizer implements Localizer {
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    public static double X = 132.5, Y = -0.6;
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.REVERSED, yPod = PinPoint.EncoderDirection.FORWARD;
    private PinPoint pinPoint;
    public PinPointLocalizer(PinPoint devie){
        pinPoint = devie;
        pinPoint.recalibrateIMU();
        pinPoint.resetPosAndIMU();
        pinPoint.setEncoderResolution(PinPoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(xPod, yPod);
        pinPoint.setOffsets(X, Y);
    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(pinPoint.getPosX(), pinPoint.getPosY(), pinPoint.getHeading());
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.getX(), pose2d.getY(), AngleUnit.RADIANS, pose2d.getHeading()));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(pinPoint.getVelX(), pinPoint.getVelY(), pinPoint.getHeadingVelocity());
    }

    @Override
    public void update() {
        pinPoint.update(DistanceUnit.INCH);

    }
}
