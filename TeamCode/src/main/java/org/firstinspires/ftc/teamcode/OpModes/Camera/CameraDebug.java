package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.exception.util.Localizable;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class CameraDebug extends LinearOpMode {
    public static boolean photo = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A camera = hardwareMap.get(Limelight3A.class, "camera");
        Localizer.Initialize(hardwareMap);
        Robot.InitializeHubs(hardwareMap);
        Robot.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        camera.start();
        camera.pipelineSwitch(0);
        LLResult r = null;
        SparkFunOTOS.Pose2D p = null;

        while (opModeIsActive()){
            Robot.clearCache();
            Localizer.Update();
            if(r == null || photo){
                r = camera.getLatestResult();
                if(r != null){
                    p = GetPositionSample.getPositionRelativeToFiled(r.getTx(), r.getTy(), Localizer.getCurrentPosition());
                }
                photo = false;
            }
            if(r != null) {
//                Robot.telemetry.addData("samplePositionRelativeToRobot", GetPositionSample.getPositionRelativeToRobot(r.getTx(), r.getTy()));
//                Robot.telemetry.addData("samplePositionRelativeToField", GetPositionSample.getPositionRelativeToFiled(r.getTx(), r.getTy()));
//                SparkFunOTOS.Pose2D p = GetPositionSample.getPositionRelativeToRobot(r.getTx(), r.getTy());
//                Robot.telemetry.addData("robot pos", "(" + p.x + ", " + p.y + ", " + Math.toDegrees(p.h) + " deg)");
                Robot.telemetry.addData("field pos", "(" + p.x + ", " + p.y + ", " + Math.toDegrees(p.h) + " deg)");
            }
            telemetry.update();
        }
    }
}
