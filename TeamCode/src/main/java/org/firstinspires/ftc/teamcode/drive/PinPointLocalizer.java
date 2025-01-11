package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class PinPointLocalizer implements Localizer {
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    public static double X = 132.5, Y = -0.6;
    private Pose2d lastPos = new Pose2d(), velo = new Pose2d();
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.REVERSED, yPod = PinPoint.EncoderDirection.FORWARD;
    private PinPoint pinPoint;
    public static boolean reset = false;
    public PinPointLocalizer(PinPoint devie){
        pinPoint = devie;
        if(!reset) {
            pinPoint.recalibrateIMU();
            reset = true;
        }
        pinPoint.resetPosAndIMU();
        pinPoint.setEncoderResolution(PinPoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(xPod, yPod);
        pinPoint.setOffsets(X, Y);
    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        double h = pinPoint.getHeading();
        while(h > Math.PI * 2) h -= Math.PI * 2;
        while(h < 0) h += Math.PI * 2;
        return new Pose2d(pinPoint.getPosX(), pinPoint.getPosY(), pinPoint.getHeading());
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        double h = pose2d.getHeading();
        while(h < -Math.PI) h += 2 * Math.PI;
        while(h > Math.PI) h -= 2 * Math.PI;
        pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.getX(), pose2d.getY(), AngleUnit.RADIANS, pose2d.getHeading()));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
//        return new Pose2d(pinPoint.getVelX(), -pinPoint.getVelY(), pinPoint.getHeadingVelocity());
        return velo;
    }
    private ElapsedTime time = new ElapsedTime();
    @Override
    public void update() {
        pinPoint.update(DistanceUnit.INCH);

        velo = new Pose2d(currentPose.getX() - lastPos.getX(), -currentPose.getY() + lastPos.getY(), currentPose.getHeading() - lastPos.getHeading());
        velo.div(time.seconds());
        time.reset();
    }
}
