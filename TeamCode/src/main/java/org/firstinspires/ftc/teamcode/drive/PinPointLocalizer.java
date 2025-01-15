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
    private static Pose2d off = new Pose2d(0, 0, 0);
    private Pose2d lastPos = new Pose2d(), velo = new Pose2d();
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.REVERSED, yPod = PinPoint.EncoderDirection.FORWARD;
    public PinPoint pinPoint;
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

    public void reset(){
        off = new Pose2d(getPoseEstimate().getX() + off.getX(), getPoseEstimate().getY() + off.getY(), getPoseEstimate().getHeading() + off.getHeading());
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        double h = pinPoint.getHeading();
        while(h > Math.PI * 2) h -= Math.PI * 2;
        while(h < 0) h += Math.PI * 2;
        return new Pose2d(pinPoint.getPosX() - off.getX(), pinPoint.getPosY() - off.getY(), pinPoint.getHeading() - off.getHeading());
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        double h = pose2d.getHeading();
        while(h < -Math.PI) h += 2 * Math.PI;
        while(h > Math.PI) h -= 2 * Math.PI;
        reset();
        off = new Pose2d(off.getX() + pose2d.getX(), off.getY() + pose2d.getY(), off.getHeading() + pose2d.getHeading());
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {

//        return new Pose2d(pinPoint.getVelX(), pinPoint.getVelY(), pinPoint.getHeadingVelocity());
        return velo;
    }
    private ElapsedTime time = new ElapsedTime();
    @Override
    public void update() {
        pinPoint.update(DistanceUnit.INCH);

//        velo = new Pose2d(currentPose.getX() - lastPos.getX(), currentPose.getY() - lastPos.getY(), currentPose.getHeading() - lastPos.getHeading());
        velo = new Pose2d(getPoseEstimate().getX() - lastPos.getX(), getPoseEstimate().getY() - lastPos.getY(), getPoseEstimate().getHeading() - lastPos.getHeading());
        velo.div(time.seconds());
        lastPos = getPoseEstimate();
        time.reset();
    }
}
