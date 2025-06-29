package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.LimeLightStream;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.HelperClasses.MJpegStreamDecoder;

@TeleOp
@Config
public class CameraDebug extends LinearOpMode {
    public static boolean photo = false;
    public static int id = 2;
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
        SparkFunOTOS.Pose2D p = null, o = null;

        while (opModeIsActive()){
            Robot.clearCache();
            Localizer.Update();
            Robot.telemetry.addData("is camera off", camera.isConnected());
            r = camera.getLatestResult();
            if(r != null && r.isValid() && GetPositionSample.hasId(r, id)){
                double tx = GetPositionSample.getOptimalResult(r, id).getTargetXDegrees(), ty = GetPositionSample.getOptimalResult(r, id).getTargetYDegrees();
                o = GetPositionSample.getPositionRelativeToRobot(tx, ty);
                Robot.telemetry.addData("robot pos", "(" + o.x + ", " + o.y + ", " + Math.toDegrees(o.h) + " deg)");
            }
            telemetry.update();
        }
    }
}
