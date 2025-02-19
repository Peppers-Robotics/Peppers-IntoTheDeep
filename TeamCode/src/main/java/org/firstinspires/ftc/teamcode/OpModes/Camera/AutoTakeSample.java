package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class AutoTakeSample extends LinearOpMode {
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
    public static boolean run = false;
    public static double pow = 0.004;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        DropDown.setDown(0);
        Extendo.Extend(0);

        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");

        run = false;
        res = null;

        Scheduler task = new Scheduler();
        Scheduler takeNew = new Scheduler();
        task
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        run = true;
                        res = ll.getLatestResult();
                        return res != null;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend((int) GetPositionSample.getExtendoRotPair(res.getTx(), res.getTy()).x);
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, GetPositionSample.getExtendoRotPair(res.getTx(), res.getTy()).h));
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
                .waitSeconds(0.05)
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
        takeNew
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        res = ll.getLatestResult();
                        capture = Localizer.getCurrentPosition();
                        return res != null;

                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        run = false;
                        return gamepad1.square;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        run = true;
                        Chassis.setTargetPosition( new SparkFunOTOS.Pose2D(
                                Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y,
//                                Localizer.getCurrentPosition().h +
                                    GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture)).h
                        ));
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Math.toDegrees(Math.abs(Localizer.getCurrentPosition().h - Chassis.getTargetPosition().h)) < 10;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend((int) GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture)).x);
                        return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 4;
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
                .waitSeconds(0.25)
                .addTask(new Task() {
                    double p = -1;
                    @Override
                    public boolean Run() {
                        if(p == -1) p = Extendo.getTargetPosition();
                        p -= pow / hz;
                        Extendo.Extend((int) (Extendo.getTargetPosition() + p));
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
                        ActiveIntake.Reverse(0.4);
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

                ;


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
                Robot.telemetry.addData("pos forward", GetPositionSample.getPositionRelativeToRobot(res.getTx(), res.getTy()).x);
                Robot.telemetry.addData("pos lateral", GetPositionSample.getPositionRelativeToRobot(res.getTx(), res.getTy()).y);
                 SparkFunOTOS.Pose2D r = GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture);
                Robot.telemetry.addData("field pos", r.x + ", " + r.y );
            }

            hz = 1 / t.seconds();
            Extendo.update();
            if(run)
                Chassis.Update();
            Localizer.Update();
            Robot.telemetry.addData("rot tp", Math.toDegrees(Chassis.getTargetPosition().h));

        }

    }
}
