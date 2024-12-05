package org.firstinspires.ftc.teamcode.HelperClasses.LimeLightHelpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.HelperClasses.Colors;

@Config
public class LimeLightColorTracking {
    public static double kPF = 0.2, kPR = 0.5;
    public static double TargetTX = 0, TargetTY = 0, TargetDistance = 0;
    public static Colors.ColorType RepresentationOfHead = Colors.ColorType.GREEN, TrackColor = Colors.ColorType.YELLOW;
    public static Limelight3A camera;
    public static void StartCamera(){
        camera.setPollRateHz(20);
        camera.pipelineSwitch(1);
        camera.start();
    }
    public static void Stop(){
        camera.stop();
    }

    public static double getTx(){
        return camera.getLatestResult().getPythonOutput()[0];
    }
    public static double moveForwardExtendo(){
        return kPF * (getDistance() - TargetDistance);
    }
    public static double rotateDegree(){
        return kPR * (getTy() - TargetTY);
    }
    public static double getTy(){
        return camera.getLatestResult().getPythonOutput()[1];
    }
    public static double getDistance(){
        return camera.getLatestResult().getPythonOutput()[2];
    }

}
