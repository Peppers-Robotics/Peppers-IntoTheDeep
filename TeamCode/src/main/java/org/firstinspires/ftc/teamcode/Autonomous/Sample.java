package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.widget.TabHost;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.Extension;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import kotlin.StandardKt;

@Autonomous(name = "0 + 7")
@Config
public class Sample extends LinearOpMode {
    public static boolean isRetracting = false;
    public static double parkElevator = 250;
    public static Limelight3A camera;
    public static PIDCoefficients headingCoeff;
    private static LLResult result = null;

    public static class Transfer extends Task{
        private Scheduler transfer;
        public Transfer(){
            transfer = new Scheduler();
            transfer
            .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        DropDown.setDown(0);
                        Claw.open();
                        Extendo.PowerOnToTransfer = true;
                        Extension.Retract();
                        Arm.setArmAngle(OutTakeLogic.ArmIdle);
                        if(Extendo.getCurrentPosition() < 40){
                            Elevator.PowerOnDownToTakeSample = true;
                        }
                        return Extendo.getCurrentPosition() < 20;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.powerOn();
                        DropDown.setDown(0);
                        Elevator.PowerOnDownToTakeSample = true;
                        Extension.Extend(OutTakeLogic.TransferExtension);
                        return Extendo.getCurrentPosition() < 20;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        ActiveIntake.Block();
                        Arm.setArmAngle(OutTakeLogic.ArmIdle);
                        DropDown.setDown(0);
                        Elevator.PowerOnDownToTakeSample = true;
                        Elevator.power = 1;
                        Extendo.PowerOnToTransfer = true;
                        Extendo.Extend(0);
                        return Elevator.getCurrentPosition() < 20;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.close();
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = false;
                        Extendo.PowerOnToTransfer = false;
//                                Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
//                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
//                        if(Elevator.getCurrentPosition() >= 5)
//                            Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        ActiveIntake.powerOff();
                        return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), basketPosition) < 400;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                        if(Elevator.getCurrentPosition() >= 20)
                            Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        return Math.abs(Elevator.getCurrentPosition() - Elevator.getTargetPosition()) < 700;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        if(Arm.getCurrentArmAngle() >= 190){
                            Extension.Extend(0.2);
                        }
                        DropDown.setDown(0);
                        return Arm.getCurrentArmAngle() > OutTakeLogic.ArmScoreSample - 15;
                    }
                })
        ;

        }
        @Override
        public boolean Run() {
            transfer.update();
            return transfer.done();
        }
    }
    public static class Retract extends Task{
        private Scheduler retract;
        public Retract(){
            retract = new Scheduler();
            retract
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Claw.close();
                            if(Extendo.getTargetPosition() < 200 && Extendo.getCurrentPosition() < 200)
                                Extendo.Extend(150);
                            Extension.Retract();
                            Arm.setArmAngle(OutTakeLogic.ArmIdle);
                            Elevator.PowerOnDownToTakeSample = true;
                            Elevator.setTargetPosition(0);
                            Elevator.power = 1;
                            isRetracting = true;
                            return true;
                        }
                    })
                    ;

        }

        @Override
        public boolean Run() {
            retract.update();
            return retract.done();
        }
    }
    private static int cnt = 0;
    public static class TakeSample extends Task {
        private SparkFunOTOS.Pose2D GlobalSample = new SparkFunOTOS.Pose2D(0,0,0);
        private Scheduler take;
        private double p = -1;
        private double tx, ty;
        private SparkFunOTOS.Pose2D took;
        public TakeSample(int id, double timeOut){
            p = -1;
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            camera.start();
                            camera.pipelineSwitch(0);
                            Claw.closeAbit();
                            ActiveIntake.Unblock();
                            Extendo.Extend(0);
                            result = null;
                            return true;
                        }
                    })
                    .waitForSync()
//                    .waitForStill()
//                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            result = camera.getLatestResult();
                            if(result != null && result.isValid() && GetPositionSample.hasId(result, id)){
                                camera.captureSnapshot("image" + cnt++);
                                headingCoeff = Chassis.Heading.getCoeff();
//                                Chassis.Heading.setPidCoefficients(Chassis.FullExtendoHeading);

                            }
//                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y, park.h));
                            return result != null && result.isValid() && GetPositionSample.hasId(result, id) /*&& Localizer.getVelocity().x < 2 && Localizer.getVelocity().y < 2 && Localizer.getAngleDifference(park.h, Localizer.getCurrentPosition().h) < Math.toRadians(2)*/;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {

                            LLResultTypes.DetectorResult res = GetPositionSample.getOptimalResult(result, id);
                            tx = res.getTargetXDegrees();
                            ty = res.getTargetYDegrees();


//                            Chassis.Heading = new PIDController(new PIDCoefficients(1.8, 0.3, 0.05));
                            Chassis.setHeading(GetPositionSample.getExtendoRotPair(tx, ty).h);
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.setHeading(GetPositionSample.getExtendoRotPair(tx, ty).h);
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(2);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            SparkFunOTOS.Pose2D extendandRotData = GetPositionSample.getExtendoRotPair(tx,ty);

                            Extendo.Extend(
                                    (int) extendandRotData.x - 18 // 30
                            );
                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 10 && Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(3);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOn(0.7);
                            return true;
                        }
                    })

                    .addTask(new Task() {
                        boolean start = false;
                        long time = -1;
                        @Override
                        public boolean Run() {

//                            Robot.telemetry.addData("target (ex, rot)", Extendo.getTargetPosition() + " " + Math.toRadians(Chassis.getTargetPosition().h));
//                            Robot.telemetry.addData("sample distance forward", GetPositionSample.getPositionRelativeToRobot(tx, ty).x);
                            if(Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) <= 10 &&
                               Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(10)){
                                start = true;
                            }
                            if(start){
                                ActiveIntake.powerOn();
                                DropDown.setDown(1);
                                if(time == -1) time = System.currentTimeMillis();
                                if(p == -1) p = Extendo.getTargetPosition();
                            }
                            if(p != -1 && (System.currentTimeMillis() - time) / 1000.f >= 0.1 && start) {
//                                Extendo.Extend(Extendo.getTargetPosition() + 120);
//                                Extendo.Extend(Extendo.getMaxPosition());
                                Extendo.Extend(1000);
//                                Chassis.setHeading(Localizer.getCurrentPosition().h + Math.toRadians(10));
                            }
                            if(((System.currentTimeMillis() - time) / 1000.f > timeOut + (ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 3 ? 0.5 : 0) && time != -1)){
                                return true;
                            }
//                            Robot.telemetry.addData("POW", ActiveIntake.motor.getCurrent(CurrentUnit.AMPS));
                            return Storage.getStorageStatus() == GetPositionSample.getType(id);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                       //     Chassis.Heading.setPidCoefficients(headingCoeff);
                            if(Storage.getStorageStatus() == GetPositionSample.getType(id)) {
                                ActiveIntake.Block();
                                DropDown.setDown(0);
                                result = null;
                                return true;
                            } else {
                                ActiveIntake.Reverse(0.6);
                                ActiveIntake.Unblock();
                            }
                            result = null;
                            camera.reloadPipeline();
                            return true;
                        }
                    })
                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.hasTeamPice())
                                ActiveIntake.Block();
                            else {
                                ActiveIntake.Unblock();
                            }
                            ActiveIntake.Reverse(0.5);
                            Extendo.Extend(0);
                            DropDown.setDown(0);
                            return true;
                        }
                    })
            ;
        }
        public TakeSample(int pos) {
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            DropDown.setDown(1);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 20 || Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        private long time = -1;
                        @Override
                        public boolean Run() {
                            if(time == -1){
                                time = System.currentTimeMillis();
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.2){
                                Extendo.Extend(pos + 100);
                            }
                            return Storage.hasTeamPice() || (double) (System.currentTimeMillis() - time) / 1000.f >= 0.7;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Extension.Extend(OutTakeLogic.TransferExtension);
                            ActiveIntake.Unblock();
//                            ActiveIntake.Block();
                            ActiveIntake.powerOn();
                            Extendo.Extend(0);
                            DropDown.setDown(0);
                            return true;
                        }
                    })
                    ;
        }
        @Override
        public boolean Run() {
            take.update();
            return take.done();
        }
    }
    public static class GoToTakeSampleFromSubmersible extends Task {
        private Scheduler go, tmp;
        private static final Scheduler takeTry = new Scheduler();
        static {
            takeTry.
                    addTask(new TakeSample(0, 0.4))
                    ;
        }
        public GoToTakeSampleFromSubmersible(){
            go = new Scheduler();
            go
//                    .lineToLinearHeadingAsync(new SparkFunOTOS.Pose2D(park.h, park.y, Math.toRadians(10)))
                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(park.x + 100, park.y, basketPosition.h), park))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getCurrentPosition().y < -250;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp = new Scheduler();
                            tmp.addTask(new TakeSample(2, 0.5));
                            Extension.Retract();
//                            Extension.Extend(OutTakeLogic.TransferExtension);
                            return true;
                        }
                    })
                    .addTask(new Retract())
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(0);
                            return true;
                        }
                    })
                    .waitForTrajDone(80)
//                    .lineToAsync(new SparkFunOTOS.Pose2D(park.x - 150, park.y, park.h))

                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp.update();
                            if(tmp.done() && !Storage.hasTeamPice()){
                                tmp.addTask(new TakeSample(2, 0.5));
                            }
                            return tmp.done();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            DropDown.setDown(0);
                            Chassis.setProfiles(5000, 5000, 10000, 10000, 900, 900);
                            return true;
                        }
                    })
//                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(park.x + 200, park.y, Math.toRadians(60)), basketPosition))
                    .lineToAsync(new SparkFunOTOS.Pose2D(park.x + 200, park.y, basketPosition.h))
                    .waitForTrajDone(80)
                    .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, basketPosition.h + Math.toRadians(5)))
                    .waitForTrajDone(70)
                    .addTask(new Transfer())
//                    .waitForTrajDone(95)
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.closeAbit();
                            Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                            Chassis.setProfiles(5000, 5000, 5000, 5000, 2000, 2000);
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(0);
                            return true;
                        }
                    })
                    ;
        }

        @Override
        public boolean Run() {
            go.update();
            return go.done();
        }
    }

    public static SparkFunOTOS.Pose2D
            basketPosition = new SparkFunOTOS.Pose2D(460, -160, Math.toRadians(55)),
            sample1 = new SparkFunOTOS.Pose2D(510, -230, Math.toRadians(65)),
            sample2 = new SparkFunOTOS.Pose2D(510, -230, Math.toRadians(85)),
            sample3 = new SparkFunOTOS.Pose2D(470, -260, Math.toRadians(110)),
            park = new SparkFunOTOS.Pose2D(-370, -1350, Math.toRadians(0)),
            parkk = new SparkFunOTOS.Pose2D(-470, -1460, Math.toRadians(0))
    ;

    private static long startTime = 0;
    private static double autoTimer = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Extendo.Extend(0);
        DropDown.setDown(0);
        Arm.setArmAngle(50);
        Elevator.setTargetPosition(0);
        Claw.close();
        Chassis.resetProfiles();
        camera = hardwareMap.get(Limelight3A.class, "camera");
        camera.start();
        camera.pipelineSwitch(0);
        Extension.Retract();
        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        Chassis.setProfiles(5000, 5000, 10000, 10000, 4000, 4000);
//        Storage.team = Storage.Team.RED;
        cnt = 0;
        camera.deleteSnapshots();
        parked = false;
        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Scheduler scheduler = new Scheduler();
        scheduler
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x, basketPosition.y, Math.toRadians(70)))
                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, Math.toRadians(70)))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                        Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        return true;
                    }
                })
//                .lineTo(basketPosition)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Elevator.getCurrentPosition() > OutTakeLogic.ElevatorScoreSample2 - 150;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        return Arm.motionCompleted();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                        DropDown.setDown(0);
                        return true;
                    }
                })
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        Extendo.Extend(700);
//                        Chassis.Heading.setPidCoefficients(new PIDCoefficients(1.5, 0.5, 0.07));
                        ActiveIntake.powerOn();
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        return true;
                    }
                })
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 10, basketPosition.y + 20, sample1.h))
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 10, basketPosition.y + 20, basketPosition.h + Math.toRadians(10)))
                .lineToAsync(basketPosition)
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.closeAbit();
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .lineToAsync(sample1)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Localizer.getCurrentPosition().h >= Math.toRadians(60)){
                            Extendo.Extend(850);
                            DropDown.setDown(1);
                        }
                        return Localizer.getCurrentPosition().h > Math.toRadians(60) && Localizer.getCurrentPosition().y < -220;
                    }
                })
                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(850))
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 10, basketPosition.y + 20, basketPosition.h - Math.toRadians(10)))
//                .lineToAsync(basketPosition)

                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, basketPosition.h - Math.toRadians(10)))
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        Extendo.Extend(0);
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        Extension.Extend(0.22);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.closeAbit();
                        return true;
                    }
                })
//                .waitSeconds(0.05)
                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        return Localizer.getCurrentPosition().y < -200;
                        if(Localizer.getCurrentPosition().h >= Math.toRadians(60)){
                            DropDown.setDown(1);
                            ActiveIntake.powerOn();
                            Extendo.Extend(850);
                        }
                        return Localizer.getCurrentPosition().h > Math.toRadians(80) && Localizer.getCurrentPosition().y < -210;
                    }
                })
                .addTask(new Retract())
//                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h > Math.toRadians(80);
                    }
                })
                .addTask(new TakeSample(820))
//                .lineToAsync(basketPosition)

                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, basketPosition.h - Math.toRadians(10)))
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 10, basketPosition.y + 20, basketPosition.h))
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 10, basketPosition.y + 20, basketPosition.h - Math.toRadians(10)))
//                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        Extension.Extend(0.22);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.closeAbit();
                        return true;
                    }
                })
                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        return Localizer.getCurrentPosition().y < -200;
                        ActiveIntake.powerOn();
//                        Extendo.Extend(600);
                        if(Localizer.getCurrentPosition().h >= Math.toRadians(85)){
                            ActiveIntake.powerOn();
                            DropDown.setDown(1);
                            Extendo.Extend(780);
                        }
                        return Localizer.getCurrentPosition().h > Math.toRadians(90);
                    }
                })
                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(850))
//                .lineToAsync(basketPosition)
                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, basketPosition.h - Math.toRadians(10)))
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        return true;
                    }
                })
//                .waitSeconds(0.05)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.closeAbit();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                        Chassis.setProfiles(6000, 6000, 6000, 6000, 2000, 2000);
                        return true;
                    }
                })
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                ;
        Chassis.Autonomous = true;
        DropDown.setDown(0);
        while(opModeInInit()){
            Robot.clearCache(false);
            if(gamepad1.square) Storage.team = Storage.Team.RED;
            if(gamepad1.circle) Storage.team = Storage.Team.BLUE;

            telemetry.addData("TEAM", Storage.team.toString());
            Extendo.update();
            Arm.update();
            Elevator.update();
            Localizer.Update();
            telemetry.update();
        }
        startTime = System.currentTimeMillis();
        autoTimer = 0;
        long auto = System.currentTimeMillis();
        LinearFunction f = new LinearFunction(0.7, 2);

        while(opModeIsActive()){
            Robot.clearCache(false);
            if(isRetracting && Elevator.getCurrentPosition() < 10) {
                Elevator.PowerOnDownToTakeSample = false;
                isRetracting = false;
            }
            if(autoTimer >= 30 - f.getOutput(Math.abs((Math.abs(park.x) - Math.abs(Localizer.getCurrentPosition().x)) / Math.abs(park.x))) && !parked &&
                    Math.abs(Localizer.getCurrentPosition().y - basketPosition.y) > 700
                    ){
                parked = true;
                scheduler = new Scheduler();
                scheduler
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                result = null;
                                ActiveIntake.Reverse(0.4);
                                ActiveIntake.Block();
                                return Localizer.getCurrentPosition().y < -300;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(park.x - 20, park.y, Math.toRadians(20)));
                                Chassis.asyncFollow = false;
                                Elevator.setTargetPosition(parkElevator);
//                                if(Elevator.getTargetPosition() > Elevator.getTargetPosition() - 120)
                                Arm.setArmAngle(110);
                                if(result != null && result.isValid() && Storage.isStorageEmpty()){
                                    LLResultTypes.DetectorResult r = GetPositionSample.getOptimalResult(result, 2);
//                                    Extendo.Extend((int) GetPositionSample.getExtendoRotPair(r.getTargetXDegrees(), r.getTargetYDegrees()).x);
                                } else {
                                    result = camera.getLatestResult();
                                }
                                return Elevator.getCurrentPosition() > Elevator.getTargetPosition() - 5;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                if(result != null && result.isValid() && Storage.isStorageEmpty()){
                                    LLResultTypes.DetectorResult r = GetPositionSample.getOptimalResult(result, 2);
                                    Extendo.Extend((int) GetPositionSample.getExtendoRotPair(r.getTargetXDegrees(), r.getTargetYDegrees()).x);
                                } else {
                                    result = camera.getLatestResult();
                                }
                                Extension.Extend(1);
                                return true;
                            }
                        })

                        ;

            } else {
                try {
                    scheduler.update();
                } catch (Exception ignored){
                }
            }

            Robot.telemetry.addData("timer", (System.currentTimeMillis() - auto) / 1000.f);
//            telemetry.addData("color", Storage.getStorageStatus().toString());
            autoTimer = (System.currentTimeMillis() - auto) / 1000.f;
            if((System.currentTimeMillis() - auto) / 1000.f >= 30) requestOpModeStop();
            Extendo.update();
            Robot.telemetry.addData("elevator pos", Elevator.getCurrentPosition());
            Robot.telemetry.addData("trajdone",Chassis.getPrecentageOfMotionDone());
            Robot.telemetry.addData("color seen...........",Storage.getStorageStatus());
            Localizer.Update();
            Chassis.Update();
            Arm.update();
            Elevator.update();
            Robot.telemetry.update();
        }
    }
    public static boolean parked = false;
}