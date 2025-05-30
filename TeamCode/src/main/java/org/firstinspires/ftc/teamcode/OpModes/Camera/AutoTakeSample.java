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

@TeleOp
@Config
public class AutoTakeSample extends LinearOpMode {
    //    public static double initialLimeLightAngle = 25, h = 170, distanceFromCameraToExtendoY = 75, distanceFromCameraToExtendoX = 80;
//    public static final double h35 = 269.1, h30 = 272.19, h25 = 275.214, h20 = 278.157, h15 = 281.043, h10 = 283.7, h5 = 286.2526, h0 = 288.638;
    public static final double spool = 16, RA = 4.75;
    public static final int CPR = 28;
    public static double hz = 1;
    public static boolean startgetsample = false;
    public static boolean starttakeserios = false;
    public static int limit_readings = 1;
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
        //DropDown.setDown(0);
        //Extendo.Extend(0);
        //Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        SparkFunOTOS.Pose2D pos_sample_field = new SparkFunOTOS.Pose2D(0,0,0);
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");

        run = false;
        res = null;

        Scheduler task = new Scheduler();
        task
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        run = true;
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
                        Extendo.Extend((int) GetPositionSample.getExtendoRotPair(tx, ty).x);
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, GetPositionSample.getExtendoRotPair(tx, ty).h));
                        return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 4;
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
        ElapsedTime t = new ElapsedTime();

        while(opModeIsActive()){
            Robot.clearCache();
            //task.update();
            run = true;
            res = ll.getLatestResult();
            Robot.telemetry.addData("camera results isValid", res!=null && res.isValid());
            if(res != null && res.isValid() && GetPositionSample.hasId(res, id)){
                LLResultTypes.DetectorResult result = GetPositionSample.getOptimalResult(res, id);
                tx = result.getTargetXDegrees();
                ty = result.getTargetYDegrees();
            }

            //Robot.telemetry.addData("tx", tx);
            //Robot.telemetry.addData("ty", ty);

            SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
            SparkFunOTOS.Pose2D normalizedPos = GetPositionSample.normalize_pos(pos);
            Robot.telemetry.addData("x robot", normalizedPos.x);
            Robot.telemetry.addData("y robot", normalizedPos.y);
            Robot.telemetry.addData("abnormal heading", pos.h);
            Robot.telemetry.addData("h robot", normalizedPos.h);
            SparkFunOTOS.Pose2D poscamera = GetPositionSample.CameraRelativeToField(pos);
            //Robot.telemetry.addData("x camera", poscamera.x);
            //Robot.telemetry.addData("y camera", poscamera.y);

            if(gamepad1.cross)
                starttakeserios = true;
            if(gamepad1.circle)
            {
                starttakeserios = false;
                startgetsample = false;
                readings = 0;
                pos_sample_field = new SparkFunOTOS.Pose2D(0,0,0);
                Chassis.setTargetPosition(pos_sample_field);
            }


            if(res != null && !startgetsample && (tx != 0 || ty != 0) && starttakeserios) {
                SparkFunOTOS.Pose2D pos_sample = GetPositionSample.getSamplePositionRelativeToCamera(tx,ty);
                //Robot.telemetry.addData("pos sample x", pos_sample.x);
                //Robot.telemetry.addData("pos sample y", pos_sample.y);
                SparkFunOTOS.Pose2D temp_pos_sample_field = GetPositionSample.getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);

                pos_sample_field.x += temp_pos_sample_field.x;
                pos_sample_field.y += temp_pos_sample_field.y;

                Robot.telemetry.addData("pos sample x relevant to field", pos_sample_field.x);
                Robot.telemetry.addData("pos sample y relevant to field", pos_sample_field.y);

                if(limit_readings == readings) {
                    startgetsample = true;
                    pos_sample_field.x /= limit_readings;
                    pos_sample_field.y /= limit_readings;
                }
                readings++;
            }

            if(res != null)
            {
                SparkFunOTOS.Pose2D pos_sample = GetPositionSample.getSamplePositionRelativeToCamera(tx,ty);
                SparkFunOTOS.Pose2D temp_pos_sample_field = GetPositionSample.getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);
                Robot.telemetry.addData("pos sample x relevant to field", temp_pos_sample_field.x);
                Robot.telemetry.addData("pos sample y relevant to field", temp_pos_sample_field.y);

            }

            SparkFunOTOS.Pose2D pleaseDo = GetPositionSample.GetExtendoTicksToTravelAndNeededAngle(normalizedPos,pos_sample_field);
            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(pos.x,pos.y, pleaseDo.h));
            Extendo.Extend((int)pleaseDo.x);
            //hz = 1 / t.seconds();

            if(startgetsample) {
                Chassis.Update();
                //Extendo.update();
            }


            //Localizer.Update();
            //Robot.telemetry.addData("rot tp", Math.toDegrees(Chassis.getTargetPosition().h));

            Robot.telemetry.addData("sample pos x", pos_sample_field.x);
            Robot.telemetry.addData("sample pos y", pos_sample_field.y);

            Robot.telemetry.update();
            Localizer.Update();

        }

    }
}
