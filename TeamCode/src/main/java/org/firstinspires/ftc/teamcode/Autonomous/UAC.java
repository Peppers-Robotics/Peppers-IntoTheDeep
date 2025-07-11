package org.firstinspires.ftc.teamcode.Autonomous;

import android.hardware.camera2.params.BlackLevelPattern;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OpModes.OpModeManager;
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

import java.util.Arrays;

@Autonomous(name = "0 + 7 ( UAC )")
@Config
public class UAC extends LinearOpMode {
    public static boolean isRetracting = false;
    public static double parkElevator = 250;
    public static Limelight3A camera;
    public static PIDCoefficients headingCoeff;
    private static LLResult result = null;
    public static class Transfer extends Task {
        private Scheduler transfer, run;
        private boolean retry = false;
        public Transfer(){
            transfer = new Scheduler();
            {
                transfer
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Elevator.setTargetPosition(0);
                                return true;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                DropDown.setDown(0);
                                Claw.open();
                                Extendo.PowerOnToTransfer = true;
                                Extendo.power = 1;
                                Extension.Retract();
                                Arm.setArmAngle(OutTakeLogic.ArmTransfer - 3);
                                if(Extendo.getCurrentPosition() < 40){
                                    Elevator.PowerOnDownToTakeSample = true;
                                }
                                return Extendo.lm.getState();
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                return true;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Robot.telemetry.addLine("next State");
                                Extendo.power = 1;
                                ActiveIntake.powerOn();
                                DropDown.setDown(0);
                                Elevator.PowerOnDownToTakeSample = true;
                                Extension.Extend(OutTakeLogic.TransferExtension);
                                return true;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.Unblock();
                                Arm.setArmAngle(OutTakeLogic.ArmTransfer - 11);
                                DropDown.setDown(0);
                                Elevator.PowerOnDownToTakeSample = true;
                                Elevator.power = 1;
                                Extendo.PowerOnToTransfer = true;
                                Extendo.Extend(0);
                                return Elevator.getCurrentPosition() <= 2;
                            }
                        })
                        .waitSeconds(0.06)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Claw.close();
                                ActiveIntake.Unblock();
                                return true;
                            }
                        })
                        .waitSeconds(0.05)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Elevator.PowerOnDownToTakeSample = false;
                                Extendo.PowerOnToTransfer = false;
                                Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                                if(Elevator.getTargetPosition() > 200)
                                    Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                                return true;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                if(Elevator.getCurrentPosition() > 200 && Storage.hasTeamPice()){
                                    retry = true;
                                }
                                return true;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOff();
                                return Math.abs(Elevator.getCurrentPosition() - Elevator.getTargetPosition()) < 300;
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
                                return Arm.motionCompleted();
                            }
                        })
                ;
                run = transfer.clone();
            }
        }
        @Override
        public boolean Run() {
            run.update();
            if(retry){
                retry = false;
                run = transfer.clone();
            }
            return run.done();
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
                            Extension.Retract();
                            Arm.setArmAngle(OutTakeLogic.ArmTransfer);
//                            Elevator.setTargetPosition(0);
                            Elevator.power = 1;
                            isRetracting = true;
                            return Arm.getCurrentArmAngle() <= 30;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.setTargetPosition(0);
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
    public static class TakeSamplePacanea extends Task{
        private Scheduler take;
        private double tx, ty;
        private SparkFunOTOS.Pose2D took;
        private double ks = 0;
        public static double offset = 5, extendo = 0;
        public TakeSamplePacanea(int id, double timeOut){
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            ActiveIntake.Unblock();
                            Extendo.Extend(0);
                            Chassis.setHeading(Math.toRadians(offset));
                            Chassis.asyncFollow = false;
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(offset >= 40) {
                                offset = 0;
                                extendo += 100;
                            }
                            else offset += 10;
                            Extendo.Extend((int) (600 + extendo));
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        boolean start = false;
                        long time = -1;
                        @Override
                        public boolean Run() {
                            if(!Storage.isStorageEmpty() && Storage.hasWrongPice()) return true;

                            if(Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) <= 40 &&
                                    Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(10)){
                                start = true;
                            }
                            if(start && time == -1){
                                ActiveIntake.powerOn();
                                DropDown.setDown(1);
                                if(time == -1) time = System.currentTimeMillis();
                                Extendo.motor.setPower(0);
                                Extendo.DISABLE = true;
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.1 && start && (System.currentTimeMillis() - time) / 1000.f < 0.2) {
                                Extendo.motor.setPower(-0.7);
                            } else if((System.currentTimeMillis() - time) / 1000.f >= 0.2 && start && (System.currentTimeMillis() - time) / 1000.f < 0.3){
                                Extendo.motor.setPower(0.7);
                            } else if((System.currentTimeMillis() - time) / 1000.f >= 0.3 && start && (System.currentTimeMillis() - time) / 1000.f < 0.4){
                                Extendo.motor.setPower(-0.7);
                            } else {
                                Extendo.motor.setPower(0.7);
                            }
                            if(((System.currentTimeMillis() - time) / 1000.f > timeOut + (ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 3 ? 0.5 : 0) && time != -1)){
                                return true;
                            }
//                            if(ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 2.5) ActiveIntake.powerOn(0.7);
//                            Robot.telemetry.addData("POW", ActiveIntake.motor.getCurrent(CurrentUnit.AMPS));
                            else return Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.DISABLE = false;
//                            Chassis.Heading.setPidCoefficients(headingCoeff);
                            if(Storage.hasTeamPice()) {
                                Robot.telemetry.addLine("Robot has a sample");
                                ActiveIntake.Block();
                                DropDown.setDown(0);
                                result = null;
                                return true;
                            } else {
                                ActiveIntake.Reverse(0.7);
                                ActiveIntake.Unblock();
                            }
                            result = null;
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Reverse(0.6);
                            Chassis.Heading.kS = ks;
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
    private static boolean stopDetection = false;
    public static class TakeSample extends Task {
        private Scheduler take;
        private double tx, ty;
        private SparkFunOTOS.Pose2D took;
        private double ks = 0, rotate = 0;
        public TakeSample(int id, double timeOut){
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            if(id == 2) { // yellow
                                camera.pipelineSwitch(2);
                            } else if(id == 0){ // blue
                                camera.pipelineSwitch(3);
                            } else  // red
                                camera.pipelineSwitch(4);
                            ActiveIntake.Unblock();
                            Extendo.Extend(0);
                            Chassis.setHeading(park.h + Math.toRadians(rotate));
                            Chassis.asyncFollow = false;
                            result = null;
                            return Extendo.getCurrentPosition() < 30;
                        }
                    })
//                    .waitForSync()
                    .waitForStill()
//                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Robot.telemetry.clearAll();
                            return true;
                        }
                    })
                    /*.addTask(new Task() {
                        long time = -1;
                        @Override
                        public boolean Run() {
                            if(time == -1) time = System.currentTimeMillis();
                            ActiveIntake.Unblock();
                            result = camera.getLatestResult();
                            took = Localizer.getCurrentPosition();
                            if(result != null && result.isValid() && GetPositionSample.hasId(result, id)){
                                camera.captureSnapshot("image" + cnt++);
                                headingCoeff = Chassis.Heading.getCoeff();
                            }
                            if(!camera.isConnected()) RobotLog.d("camera disconnected");
                            if(!camera.isRunning()) RobotLog.d("camera stopped");
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.4){
                                take.clear();
                                stopDetection = true;
                                return true;
                            }
                            return result != null && result.isValid() && GetPositionSample.hasId(result, id);
                        }
                    })*/
                    .addTask(new Task() {
                        long time = -1;
                        @Override
                        public boolean Run() {

                            if(time == -1) time = System.currentTimeMillis();
                            ActiveIntake.Unblock();
                            result = camera.getLatestResult();
                            if(!camera.isConnected()) RobotLog.d("camera disconnected");
                            if(!camera.isRunning()) RobotLog.d("camera stopped");
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.8){
                                take.clear();
                                stopDetection = true;
                                return true;
                            }
                            return result != null && result.isValid();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            took = Localizer.getCurrentPosition();
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tx = result.getTx();
                            ty = result.getTy();
                            ks = Chassis.Heading.kS;
                            Chassis.Heading.kS = -0.08; // -0.07
                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(
                                    took.x,
                                    took.y,
                                    took.h +
                                            GetPositionSample.getExtendoRotPair(tx, ty).h
                            ));
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(5);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(
                                    (int) GetPositionSample.getExtendoRotPair(
                                            tx, ty
                                    ).x - 20 // -70
                            );
                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 30 && Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(3);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOn(1);
                            return true;
                        }
                    })

                    .addTask(new Task() {
                        boolean start = false;
                        long time = -1, time2 = -1;
                        @Override
                        public boolean Run() {
                            if(!Storage.isStorageEmpty() && Storage.hasWrongPice()) return true;

                            if(Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) <= 40 &&
                                    Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(10)){
                                start = true;
                            }
                            if(start && time == -1){
                                ActiveIntake.powerOn();
                                DropDown.setDown(1);
                                if(time == -1) time = System.currentTimeMillis();
                                Extendo.motor.setPower(0);
                                Extendo.DISABLE = true;
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.15 && start && (System.currentTimeMillis() - time) / 1000.f < 0.2) {
                                Extendo.motor.setPower(-1);
                            } else if((System.currentTimeMillis() - time) / 1000.f >= 0.2 && start){
                                Extendo.motor.setPower(0.6);
                            }
                            if(((System.currentTimeMillis() - time) / 1000.f > timeOut + (ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 3 ? 0.5 : 0) && time != -1)){
                                return true;
                            }
//                            if(ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 2.5) ActiveIntake.powerOn(0.7);
//                            Robot.telemetry.addData("POW", ActiveIntake.motor.getCurrent(CurrentUnit.AMPS));
                            else return Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.DISABLE = false;
//                            Chassis.Heading.setPidCoefficients(headingCoeff);
                            if(Storage.hasTeamPice()) {
                                Robot.telemetry.addLine("Robot has a sample");
                                ActiveIntake.Block();
                                DropDown.setDown(0);
                                result = null;
                                return true;
                            } else {
                                ActiveIntake.Reverse(0.7);
                                ActiveIntake.Unblock();
                            }
                            result = null;
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Reverse(0.6);
                            Chassis.Heading.kS = ks;
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
                            return Extendo.getCurrentPosition() > pos - 40 || Storage.hasTeamPice();
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
                            return Storage.hasTeamPice() || (double) (System.currentTimeMillis() - time) / 1000.f >= 0.8 + (ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) >= 3? 1 : 0) * 0.2;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(!Storage.hasTeamPice())
                                ActiveIntake.Unblock();
                            else
                                ActiveIntake.Block();
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
//            if(Storage.hasWrongPice() && Storage.getStorageStatus() != Storage.SpecimenType.NONE){
//                take.clear();
//                return true;
//            }
            return take.done();
        }
    }
    private static boolean cut = false;
    public static class GoToTakeSampleFromSubmersible extends Task {
        private Scheduler go, tmp;
        public GoToTakeSampleFromSubmersible(){
            go = new Scheduler();
            go
                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(park.x + 230 * (cut ? 0 : 1), park.y, basketPosition.h), park))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            cut = false;
                            return Localizer.getCurrentPosition().y < -250;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp = new Scheduler();
                            tmp.addTask(new Task() {
                                @Override
                                public boolean Run() {
                                    return true;
                                }
                            });
                            Extension.Retract();
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
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp.update();
                            if(tmp.done() && !Storage.hasTeamPice()){
                                if(camera.isConnected() && !stopDetection)
                                    tmp.addTask(new TakeSample(2, 0.8));
                                else {
                                    tmp.addTask(new TakeSamplePacanea(2, 0.7));
                                    stopDetection = false;
                                }
                            }
                            return tmp.done();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            DropDown.setDown(0);
                            Chassis.setProfiles(5000, 5000, 10000, 10000, 800, 800);
                            return true;
                        }
                    })
                    .lineToAsync(new SparkFunOTOS.Pose2D(park.x + 200, park.y, basketPosition.h))
                    .waitForTrajDone(80)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return true;
                        }
                    })
                    .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x, basketPosition.y + 30, basketPosition.h + Math.toRadians(5)))
                    .waitForTrajDone(60)
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
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                            Chassis.setProfiles(5000, 5000, 5000, 5000, 1000, 1000);
                            return true;
                        }
                    })
//                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(OutTakeLogic.ArmIdle);
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
    public static SparkFunOTOS.Pose2D human = new SparkFunOTOS.Pose2D(-1300, -80, Math.toRadians(0)),
                                        basketPosHuman = new SparkFunOTOS.Pose2D(440, -55, Math.toRadians(20));

    public static class GoTakeFromHuman extends Task{
        Scheduler go;
        public GoTakeFromHuman(){
            go = new Scheduler();
            go
                    .lineToAsync(human)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(0);
                            return Localizer.getCurrentPosition().x < 200; // TODO: tune
                        }
                    })
                    .addTask(new Retract())
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOn();
                            return Localizer.getCurrentPosition().x < -400;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            DropDown.setDown(1);
                            Extendo.Extend(700);
                            return Localizer.getCurrentPosition().h <= Math.toRadians(5);
                        }
                    })
                    .addTask(new TakeSample(900))
                    .lineToAsync(basketPosHuman)
                    .waitForTrajDone(80)
                    .addTask(new Transfer())
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            return true;
                        }
                    })
                    .waitSeconds(0.05)

                    ;
        }
        @Override
        public boolean Run() {
            go.update();
            return go.done();
        }
    }

    public static SparkFunOTOS.Pose2D
            basketPosition = new SparkFunOTOS.Pose2D(520, -160, Math.toRadians(55)),
            basketPositionOne = new SparkFunOTOS.Pose2D(540, -90, Math.toRadians(62)),
            basketPositionTwo = new SparkFunOTOS.Pose2D(563, -180, Math.toRadians(82)),
            sample1 = new SparkFunOTOS.Pose2D(450, -290, Math.toRadians(68)), // 68
            sample2 = new SparkFunOTOS.Pose2D(556, -286, Math.toRadians(80)),
            sample3 = new SparkFunOTOS.Pose2D(470, -260, Math.toRadians(110)),
            park = new SparkFunOTOS.Pose2D(-400, -1400, Math.toRadians(0))
                    ;

    private static long startTime = 0;
    private static double autoTimer = 0, angleToHead = 0, p = -1;
    private static boolean autoTake = false, first = false;
    @Override
    public void runOpMode() throws InterruptedException {
        first = false;
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
        angleToHead = 0;
        camera.deleteSnapshots();
        parked = false;
        autoTake = false;
        Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevator.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Scheduler scheduler = new Scheduler();
        scheduler
//                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, Math.toRadians(60)))
                .lineToAsync(basketPositionOne)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.Unblock();
                        autoTake = true;
                        first = true;
                        angleToHead = basketPositionOne.h;
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                        Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        return true;
                    }
                })
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
                        Claw.openWide();
                        return true;
                    }
                })
                .lineToAsync(sample1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h >= Math.toRadians(58) && Localizer.getCurrentPosition().y < -215 && Localizer.getCurrentPosition().h <= 70;
                    }
                })
                .addTask(new Retract())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return !autoTake;
                    }
                })

                .lineToAsync(basketPositionTwo)
                .addTask(new Transfer())
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        autoTake = true;
                        angleToHead = basketPositionTwo.h;
                        return true;
                    }
                })
//                .waitForSync()
                .waitForTrajDone(95)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        Extension.Extend(0.15);
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.openWide();
                        return true;
                    }
                })
//                .waitSeconds(0.05)
                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -240;
                    }
                })
                .addTask(new Retract())
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return !autoTake;
                    }
                })
                .lineToAsync(basketPositionTwo)
                .addTask(new Transfer())
                .waitForTrajDone(95)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 20);
                        Extension.Extend(0.15);
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.openWide();
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
                .addTask(new TakeSample(900))
//                .lineToAsync(basketPosition)
                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x + 20, basketPosition.y + 30, basketPosition.h - Math.toRadians(10)))
                .addTask(new Transfer())
//                .waitForSync()
                .waitForTrajDone(95)
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
                        Claw.openWide();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                        Chassis.setProfiles(6000, 6000, 6000, 6000, 2000, 2000);
                        return true;
                    }
                })
                .addTask(new GoTakeFromHuman())
                .addTask(new GoTakeFromHuman())
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

        Scheduler take = new Scheduler(), takeUp = new Scheduler();
        take
//                .waitForStill()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h >= angleToHead - Math.toRadians(3);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(800 + 60 * (first ? 0 : 1));
                        DropDown.setDown(1);
                        ActiveIntake.powerOn();
                        if(Arm.getCurrentArmAngle() <= 120)
                            ActiveIntake.Unblock();
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(p == -1) p = System.currentTimeMillis();
                        if(Arm.getCurrentArmAngle() <= 120)
                            ActiveIntake.Unblock();
                        return Storage.hasTeamPice() || (System.currentTimeMillis() - p) / 1000.f >= 0.9;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        p = -1;
//                        ActiveIntake.Block();
                        Extendo.Extend(0);
                        autoTake = false;
                        return true;
                    }
                })

        ;
        takeUp = take.clone();

        Storage.team = Storage.Team.RED;
        DropDown.setDown(0);
        Elevator.RESET = false;
        while(opModeInInit()){
            Robot.clearCache(true);
            if(gamepad1.square) Storage.team = Storage.Team.RED;
            if(gamepad1.circle) Storage.team = Storage.Team.BLUE;

            telemetry.addData("TEAM", Storage.team.toString());
            Extendo.update();
            Arm.update();
            Elevator.update();
            telemetry.update();
        }
        startTime = System.currentTimeMillis();
        autoTimer = 0;
        long auto = System.currentTimeMillis();
        LinearFunction f = new LinearFunction(0.5, 1.5);

        while(opModeIsActive()){
            if(autoTake && !takeUp.done()){
                takeUp.update();
                if(takeUp.done()){
                    takeUp = take.clone();
                    autoTake = false;
                }
            }
            Robot.clearCache(true);
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
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(park.x - 100, park.y, Math.toRadians(0)));
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
                                if(Arm.getCurrentArmAngle() > Arm.getArmTargetPosition() - 20) Extension.Extend(1);
                                return Elevator.getCurrentPosition() > Elevator.getTargetPosition() - 10;
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
//            Robot.telemetry.addData("ll temperature", camera.getStatus().getTemp());
//            telemetry.addData("color", Storage.getStorageStatus().toString());
            autoTimer = (System.currentTimeMillis() - auto) / 1000.f;
            if((System.currentTimeMillis() - auto) / 1000.f >= 30) requestOpModeStop();
            if(parked && Arm.getCurrentArmAngle() > 90) Extension.Extend(1);
            Extendo.update();
            Localizer.Update();
            Chassis.Update();
            Arm.update();
            Elevator.update();
            telemetry.update();
        }
    }
    public static boolean parked = false;
}
