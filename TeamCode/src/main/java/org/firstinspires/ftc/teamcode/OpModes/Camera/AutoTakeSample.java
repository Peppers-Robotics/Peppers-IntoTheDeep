package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

import java.util.Vector;

@TeleOp
@Config
public class AutoTakeSample extends LinearOpMode {
    //    public static double initialLimeLightAngle = 25, h = 170, distanceFromCameraToExtendoY = 75, distanceFromCameraToExtendoX = 80;
//    public static final double h35 = 269.1, h30 = 272.19, h25 = 275.214, h20 = 278.157, h15 = 281.043, h10 = 283.7, h5 = 286.2526, h0 = 288.638;
    public static final double spool = 16, RA = 4.75;
    public static final int CPR = 28;
    public static int offset = 18;
    public static double hz = 1;
    public static boolean startgetsample = false;
    public static boolean starttakeserios = false;
    public static int limit_readings = 1;
    public Vector<SparkFunOTOS.Pose2D>poses;
    public int readings = 0;



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
    public static SparkFunOTOS.Pose2D capture = new SparkFunOTOS.Pose2D();
    public static LLResult res = null;
    public static LLResultTypes.DetectorResult result = null;
    public static boolean run = false;
    public static double pow = 0.004;
    public static double tx, ty;
    public static int id = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        DropDown.setDown(0);
        Extendo.Extend(0);
        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        Chassis.Autonomous = true;
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");

        run = false;
        res = null;

        Scheduler task = new Scheduler();
        task
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        res = ll.getLatestResult();
                        if(res != null && res.isValid() && GetPositionSample.hasId(res, id)){
                            tx = GetPositionSample.getOptimalResult(res, id).getTargetXDegrees();
                            ty = GetPositionSample.getOptimalResult(res, id).getTargetYDegrees();
                        }
                        return res != null && res.isValid() && GetPositionSample.hasId(res, id);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, GetPositionSample.getExtendoRotPair(tx, ty).h));
                        run = true;
                        return Localizer.getAngleDifference(Chassis.getTargetPosition().h, Localizer.getCurrentPosition().h) < Math.toRadians(2);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend((int) GetPositionSample.getExtendoRotPair(tx, ty).x - offset);
                        return Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) < 20;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Unblock();
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(Extendo.getMaxPosition());
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Storage.hasTeamPice();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Block();
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Reverse(0.8);
                        DropDown.setDown(0);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        return true;
                    }
                })
        ;
        res = null;


        waitForStart();
        ll.start();
        ll.pipelineSwitch(0);

        while(opModeIsActive()){
            Robot.clearCache();
            task.update();
            res = ll.getLatestResult();

            if(run){
                Chassis.Update();
            }
            Extendo.update();

            Robot.telemetry.update();
            Localizer.Update();

        }

    }
}
