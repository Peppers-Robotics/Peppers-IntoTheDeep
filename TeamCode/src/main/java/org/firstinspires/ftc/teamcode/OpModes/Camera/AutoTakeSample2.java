package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class AutoTakeSample2 extends LinearOpMode {
//    public static double initialLimeLightAngle = 25, h = 170, distanceFromCameraToExtendoY = 75, distanceFromCameraToExtendoX = 80;
//    public static final double h35 = 269.1, h30 = 272.19, h25 = 275.214, h20 = 278.157, h15 = 281.043, h10 = 283.7, h5 = 286.2526, h0 = 288.638;
    public static final double spool = 16, RA = 4.75;
    public static final int CPR = 28;
    public static double hz = 1;

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
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        DropDown.setDown(0);
        Extendo.Extend(0);
        Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");

        run = false;
        res = null;

        Scheduler task = new Scheduler();
        task
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        res = ll.getLatestResult();
                        return res != null && res.isValid() && GetPositionSample.hasId(res, id) && gamepad1.circle;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        capture = Localizer.getCurrentPosition();
                        tx = GetPositionSample.getOptimalResult(res, id).getTargetXDegrees();
                        ty = GetPositionSample.getOptimalResult(res, id).getTargetYDegrees();
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return gamepad1.square;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y,
                                GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(tx, ty, capture), Localizer.getCurrentPosition()).h
                        ));
                        run = true;
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getAngleDifference(Chassis.getTargetPosition().h, Localizer.getCurrentPosition().h) < Math.toRadians(5) &&
                                Localizer.getAngleDifference(Localizer.getVelocity().h, 0) < Math.toRadians(3);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(
                                (int) GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(tx, ty, capture), Localizer.getCurrentPosition()).x - 35
                        );
                        Robot.telemetry.addData("Extendo TP", Extendo.getTargetPosition());
                        return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 10;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Unblock();
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Storage.getStorageStatus() == GetPositionSample.getType(id);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Block();
                        DropDown.setDown(0);
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        ActiveIntake.Reverse(0.5);
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
            task.update();
            if(res != null) {
//                Robot.telemetry.addData("rot", GetPositionSample.getExtendoRotPair(res.getTx(), res.getTy()).h);
//                Robot.telemetry.addData("ext", GetPositionSample.getExtendoRotPair(res.getTx(), res.getTy()).x);
                 SparkFunOTOS.Pose2D r = GetPositionSample.getPositionRelativeToFiled(tx, ty, capture);
                Robot.telemetry.addData("field pos", r.x + ", " + r.y );
            }

            hz = 1 / t.seconds();
            Extendo.update();
            if(run) {
                Chassis.Update();
            }
            Localizer.Update();
            Robot.telemetry.addData("rot tp", Math.toDegrees(Chassis.getTargetPosition().h));

        }

    }
}
