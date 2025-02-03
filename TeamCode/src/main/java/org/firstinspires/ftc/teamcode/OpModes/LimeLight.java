package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class LimeLight extends LinearOpMode {
    public static double initialLimeLightAngle = 25, h = 170, distanceFromCameraToExtendo = 150;
    public static final double h35 = 269.1, h30 = 272.19, h25 = 275.214, h20 = 278.157, h15 = 281.043, h10 = 283.7, h5 = 286.2526, h0 = 288.638;
    public static final double spool = 16, RA = 4.75;
    public static final int CPR = 28;

    public static int DistanceToExtendo(double d){
        return (int) (d / (2 * Math.PI * spool / RA)) * CPR;
    }

    /*

    1t .... 2*PI*spool / RA mm
    xt? .... d
    x = d /
     */
    public static double ExtendoToDistance(int e){
        return (2 * Math.PI * spool / RA) * ((double) e / CPR);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeLocalizer(hardwareMap);
        Robot.InitializeChassis();
        Robot.InitializeExtendo();
        Robot.InitializeDropDown();
        Robot.InitializeActiveIntake();
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");

        waitForStart();
        ll.start();
//        ll.pipelineSwitch(0);
        LLResult firstDetection = null;

        while(opModeIsActive()){
            if(firstDetection == null) {
                firstDetection = ll.getLatestResult();
            }
            if(firstDetection != null){
                double ty = firstDetection.getTy();
                double cameraSample = -h25 / Math.tan(Math.toRadians(90 - initialLimeLightAngle + ty));

                double tx = firstDetection.getTx();
                Extendo.Extend(DistanceToExtendo(cameraSample - distanceFromCameraToExtendo));
                Robot.telemetry.addData("distance", cameraSample);
                if(Extendo.getTargetPosition() > Extendo.getMaxPosition() - 1){
                    cameraSample -= ExtendoToDistance(Extendo.getTargetPosition());
                }
                if(cameraSample < 0){
                    cameraSample = 0;
                }
//                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(cameraSample, 0, Localizer.getCurrentPosition().h + tx));
            }
            Extendo.update();
            Chassis.Update();
            Robot.clearCache();
            DropDown.setDown(0);
        }

    }
}
