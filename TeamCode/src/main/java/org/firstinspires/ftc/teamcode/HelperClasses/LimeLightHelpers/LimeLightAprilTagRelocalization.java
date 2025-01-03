package org.firstinspires.ftc.teamcode.HelperClasses.LimeLightHelpers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class LimeLightAprilTagRelocalization {
    public static Limelight3A camera;
    public static void StartCamera(){
        if(camera.isRunning()) {
            camera.stop();
            camera.setPollRateHz(20);
        }
        camera.pipelineSwitch(0);
        camera.start();
    }
    public static Pose2d GetCurrentStartPosition(){
        LLResult result = camera.getLatestResult();
        if(!result.isValid()){
            return new Pose2d(0, 0, 0);
        }
        Pose3D pose = result.getBotpose_MT2();
        return new Pose2d(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS));
    }
    public static boolean Relocalize(SampleMecanumDrive drive){
        camera.updateRobotOrientation(Math.toDegrees(drive.getLocalizer().getPoseEstimate().getHeading()));
        LLResult result = camera.getLatestResult();
        if(!result.isValid()){
            return false;
        }

        Pose3D pose = result.getBotpose_MT2();
        drive.setPoseEstimate(new Pose2d(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS)));

        return true;
    }
    public static boolean Relocalize(){
        camera.updateRobotOrientation(Chassis.getYaw(AngleUnit.RADIANS));
        LLResult result = camera.getLatestResult();
        if(!result.isValid()) return false;

        Pose3D pose = result.getBotpose_MT2();
        Chassis.localizer.setPoseEstimate(new Pose2d(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS)));

        return true;
    }
}
