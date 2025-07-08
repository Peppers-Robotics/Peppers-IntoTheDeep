package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.CachedMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
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
import java.util.Arrays;

@Autonomous(name = "5 + 0")
@Config
public class Specimen extends LinearOpMode {
    public static class retractAsyncHelper extends Task{
        private Scheduler s;
        public retractAsyncHelper(){
            s = new Scheduler();
            s
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            samplesScored ++;
                            return true;
                        }
                    })
                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 20);
                            Extension.Extend(0);
                            return Arm.getCurrentArmAngle() >= 180;
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
            s.update();
            return s.done();
        }
    }
    public static double scoredLine = -620;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(-1000, -210, Math.toRadians(10)),
            scoreSpecimen1 = new SparkFunOTOS.Pose2D(-1000, -200, Math.toRadians(0)),
            sample1 = new SparkFunOTOS.Pose2D(-500, 860, Math.toRadians(-31)), // -33
            sample2 = new SparkFunOTOS.Pose2D(-500, 860, Math.toRadians(-50)),
            sample3 = new SparkFunOTOS.Pose2D(-500, 950, Math.toRadians(-53)),
            humanReverse = new SparkFunOTOS.Pose2D(-530, 860, Math.toRadians(-130)),
            spitDetection = new SparkFunOTOS.Pose2D(-627, 430, Math.toRadians(-140)),
            humanTake = new SparkFunOTOS.Pose2D(60, 780 ,Math.toRadians(0));
    public static int samplesScored = 0;
    public static int type = 2;
    private static int tries = 0;
    private static boolean skip = false;

    public static class ScoreSpecimen1 extends Task{
        private final Scheduler r;
        private Scheduler s;
        public ScoreSpecimen1(){
            r = new Scheduler();

            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.close();
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = false;
                            Arm.setArmAngle(OutTakeLogic.ArmScoreSpecimen);
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen + 40);
//                            return Arm.getCurrentArmAngle() < 300;
                            return true;
                        }
                    })
                    .lineToAsync(scoreSpecimen1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Arm.getCurrentArmAngle() <= OutTakeLogic.ArmScoreSample - 50){
                                Extension.Extend(1);
                            }
                            return Arm.getCurrentArmAngle() <= OutTakeLogic.ArmScoreSample - 50;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            if(samplesScored == 0){
//                                return Localizer.getCurrentPosition().x < -710;
//                            }
                            return Localizer.getCurrentPosition().x < -550; // TODO: change if needed
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            Chassis.stopFollow();
                            Chassis.drive(0, 0.3, 0);
                            return true;
                        }
                    })
//                    .lineToAsync(Localizer.getCurrentPosition())
                    .addTask(new retractAsyncHelper())
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.drive(0, 0, 0);
                            return true;
                        }
                    })
//                    .waitSeconds(0.2)
//                    .addTask(new Sample.TakeSample(type, 0.5))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.startFollow();
                            if(tries >= 2){
                                tries ++;
                                r.skip();
                                return true;
                            }
                            if(s == null){
                                s = new Scheduler();
                                s.addTask(new Sample.TakeSample(type, 1))
                                        .addTask(new Task() {
                                            @Override
                                            public boolean Run() {
                                                tries ++;
                                                return true;
                                            }
                                        });
//                                Sample.TakeSamplePacanea.offset = 15;
                                Sample.park.h = Math.toRadians(-8);
                            }
                            s.update();
                            if(s.done() && Storage.getStorageStatus() != GetPositionSample.getType(type)){
                                s = new Scheduler();
                                s.addTask(new Sample.TakeSample(type, 1))
                                        .addTask(new Task() {
                                            @Override
                                            public boolean Run() {
                                                ActiveIntake.Reverse(0.4);
                                                return true;
                                            }
                                        })
                                        .addTask(new Task() {
                                            @Override
                                            public boolean Run() {
                                                tries ++;
                                                return true;
                                            }
                                        });
                            }
                            return s.done();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.getStorageStatus() != GetPositionSample.getType(type)){
                                ActiveIntake.Unblock();
                                ActiveIntake.Reverse(0.7);
                            }
                            Extendo.Extend(0);
                            return Extendo.getCurrentPosition() < 40;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.isStorageEmpty()){
                                skip = true;
                                sample1.h = Math.toRadians(-22);
                                Extendo.Extend(0);
                            }
                            return true;
                        }
                    })
                    .lineToAsync(spitDetection)
//                    .splineToAsync(Arrays.asList(spitDetection, sample1))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, spitDetection.h) < Math.toRadians(50);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(600);
                            ActiveIntake.Reverse(1);
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, spitDetection.h) < Math.toRadians(30);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return !Storage.isStorageEmpty();
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOff();
                            Extendo.Extend(0);
                            return true;
                        }
                    })
//                    .addTask(new SpitToHP(850, 0.8))
            ;
        }
        @Override
        public boolean Run() {
            r.update();
            if(skip) return true;
            return r.done();
        }
    }

    public static class ScoreSpecimen extends Task{
        private final Scheduler r;
        public ScoreSpecimen(double o){
            r = new Scheduler();

            r
                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(-740, scoreSpecimen.y - o, Math.toRadians(50)), new SparkFunOTOS.Pose2D(scoreSpecimen.x, scoreSpecimen.y - o, scoreSpecimen.h)))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = false;
                            Extension.Extend(0);
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen - 30);
//                            return Arm.getCurrentArmAngle() < 300;
                            return Elevator.getCurrentPosition() >= 50;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.close();
                            Arm.setArmAngle(OutTakeLogic.ArmScoreSpecimen + 4);
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Arm.getCurrentArmAngle() <= OutTakeLogic.ArmScoreSample - 50){
                                Extension.Extend(1);
                            }
                            return Arm.getCurrentArmAngle() <= OutTakeLogic.ArmScoreSample - 50;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            if(Localizer.getCurrentPosition().x <= scoredLine - 150) Chassis.hProfile.setInstant(0);
                            return Localizer.getCurrentPosition().x <= scoredLine; // TODO: change if needed
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            samplesScored ++;
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                            Extension.Extend(0);
                            return Arm.getCurrentArmAngle() >= 180;
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
            r.update();
            return r.done();
        }
    }
    public static class SpecimenTake extends Task{
        private final Scheduler r;
        public SpecimenTake(boolean powerLift){
            r = new Scheduler();
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 7);
                            Elevator.setTargetPosition(0);
                            Extension.Extend(OutTakeLogic.TakeSpecimenExtension);
                            return Localizer.getCurrentPosition().x >= -10 ||
                                    (Localizer.getCurrentPosition().x >= -50 && Math.abs(Localizer.getVelocity().x) <= 200);
                        }
                    })
//                    .waitForTrajDone(70)
//                    .lineToAsync(humanTake)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = powerLift;
                            Elevator.power = 1;
                            return true;
                        }
                    })
//                    .waitForTrajDone(90)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Elevator.PowerOnDownToTakeSample = powerLift;
//                            return Elevator.getCurrentPosition() < 50;
                            if(!powerLift){
                                r.clear();
                                return true;
                            }
                            return true;
                        }
                    })
//                    .waitForTrajDone(powerLift ? 99.8 : 1)
                    .waitSeconds(0.06)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.closeAbit();
                            return true;
                        }
                    })
                    .waitSeconds(0.08)
            ;
        }

        @Override
        public boolean Run() {
            r.update();
            return r.done();
        }
    }
    public static class TakeSample extends Task{
        private final Scheduler r;
        public TakeSample(int pos){
            r = new Scheduler();
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
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
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.1){
                                Extendo.Extend(pos + 100);
                            }
                            return Storage.hasTeamPice() || (double) (System.currentTimeMillis() - time) / 1000.f >= 0.35;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(300);
                            ActiveIntake.Block();
                            return true;
                        }
                    })
                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOff();
                            return true;
                        }
                    })

            ;
        }

        @Override
        public boolean Run() {
            r.update();
            return r.done();
        }
    }
    public static class SpitToHP extends Task{
        private final Scheduler r;

        public SpitToHP(int pos, double pow){
            r = new Scheduler();
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(pos);
                            if(pow < 1){
                                return Localizer.getCurrentPosition().h < Math.toRadians(-130);
                            }
                            return Localizer.getCurrentPosition().h < Math.toRadians(-95);
//                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) < Math.toRadians(10);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            ActiveIntake.Reverse(pow);
                            return true;
                        }
                    })
                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOff();
                            return true;
                        }
                    })
            ;
        }

        @Override
        public boolean Run() {
            r.update();
            return r.done();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Scheduler auto = new Scheduler();

        Elevator.setTargetPosition(0);
        Arm.setArmAngle(70);
        Claw.close();
        tries = 0;

        Extendo.Extend(0);
        Extension.Extend(0);

        Chassis.setProfiles(8000, 8000, 10000, 10000, 3000, 3000);
        Chassis.setHeadingProfiles(20*Math.PI, 15*Math.PI, 100*Math.PI);
        Chassis.resetProfiles();

        samplesScored = 0;

        Sample.camera = hardwareMap.get(Limelight3A.class, "camera");
        Sample.camera.start();
        Sample.camera.pipelineSwitch(0);

        auto
//                .addTask(new ScoreSpecimen())
                .addTask(new ScoreSpecimen1())
                .lineToAsync(sample1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        return true;
                    }
                })
//                .waitForSync()
//                .lineToAsync(new SparkFunOTOS.Pose2D(sample1.x, sample1.y, skip ? Math.toRadians(-5) : sample1.h))
                /*.addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        if(skip){
                            return Localizer.getCurrentPosition().h <= Math.toRadians(-10);
                        } else {
                            return Localizer.getCurrentPosition().h <= Math.toRadians(-30) && Localizer.getCurrentPosition().y > 800;
                        }
                    }
                })*/
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(!skip){
                            Extendo.Extend(300);
                        }
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return (Localizer.getCurrentPosition().h >= Math.toRadians(-55) && !skip) || (skip && Localizer.getCurrentPosition().h <= Math.toRadians(-23));
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(300);
                        return true;
                    }
                })
//                .waitForTrajDone(98)
//                .waitForSync()
                .addTask(new TakeSample(680))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(500, 1))

                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        Extendo.Extend(300);
                        sample1.h = Math.toRadians(-33);
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-65);
                    }
                })
                .addTask(new TakeSample(685))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(500, 1))

                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h > Math.toRadians(-110);
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        ActiveIntake.powerOn(1);
                        return true;
                    }
                })
                .addTask(new TakeSample(850))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(500, 1))
//                .lineToAsync(humanTake)
//                .lineToAsync(new SparkFunOTOS.Pose2D(humanTake.x - 120, humanTake.y + 50, 0))
//                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 300, humanTake.y + 60, 0), humanTake))
                .lineToAsync(new SparkFunOTOS.Pose2D(-100, humanTake.y, 0))

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
                        Claw.openWide();
                        Extendo.Extend(0);
//                        Chassis.setProfiles(3000, 3000, 7000, 7000, 800, 800);
                        return Math.abs(Localizer.getCurrentPosition().h) <= Math.toRadians(10);
                    }
                })
                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(-120))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, Math.toRadians(45)), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(-100))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, Math.toRadians(45)), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(-80))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, Math.toRadians(45)), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(-60))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, Math.toRadians(45)), humanTake))
                .addTask(new SpecimenTake(tries >= 3))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(tries >= 3) requestOpModeStop();
                        return true;
                    }
                })
                .addTask(new ScoreSpecimen(-40))

//                .lineToLinearHeadingAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, Math.toRadians(45)), humanTake))
                .addTask(new SpecimenTake(false))
//                .addTask(new ScoreSpecimen())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        requestOpModeStop();
                        return true;
                    }
                })

        ;
        type = 1;
        Storage.team = Storage.Team.RED;
        Extendo.motor.setPower(-1);
        DropDown.setDown(0);

        while(opModeInInit()){

            if(gamepad1.square) {
                Storage.team = Storage.Team.RED;
                type = 1;
            }
            if(gamepad1.circle) {
                Storage.team = Storage.Team.BLUE;
                type = 0;
            }
            telemetry.addData("TEAM", Storage.team.toString());
            telemetry.update();
            if(Extendo.lm.getState()){
                Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Extendo.motor.setPower(0);
            }

            Elevator.update();
            Extendo.update();
            Arm.update();
            Robot.clearCache();
            DropDown.setDown(0);
        }
        long time = System.currentTimeMillis();

        while (opModeIsActive()) {
            auto.update();
            Robot.telemetry.addData("timer", (System.currentTimeMillis() - time) / 1000.f);
            Robot.telemetry.addData("scored", samplesScored);

            if((System.currentTimeMillis() - time) / 1000.f >= 30){
                requestOpModeStop();
            }

            Elevator.update();
            Extendo.update();
            Arm.update();
            Localizer.Update();
            Chassis.Update();
            Robot.clearCache();
        }

    }
}