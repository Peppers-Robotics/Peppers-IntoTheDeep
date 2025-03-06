package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;

import java.util.Objects;

@TeleOp
public class CameraCalibration extends LinearOpMode {
    private static double mean = 0;
    private static long samples = 0;
    private static double realTargetValue = 600;
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");
        ll.start();
        ll.pipelineSwitch(0);
        LLResult res = null;

        mean = 0;
        samples = 0;
        while(opModeInInit()) {
            telemetry.addLine("Put a sample at " + realTargetValue + " mm away from camera lens");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            res = ll.getLatestResult();
            if(res != null){
                mean += Math.atan(realTargetValue / GetPositionSample.h);
                samples ++;
            }
        }
        mean /= samples;
        telemetry.addData("effective angle", mean);
        telemetry.update();
    }
}
