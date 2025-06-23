package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
                            Elevator.setTargetPosition(0);
                            return true;
                        }
                    });
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
    public static double scoredLine = -660;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(-1000, -280, Math.toRadians(0)),
            scoreSpecimen1 = new SparkFunOTOS.Pose2D(-1000, -200, Math.toRadians(0)),
            sample1 = new SparkFunOTOS.Pose2D(-500, 860, Math.toRadians(-37)),
            sample2 = new SparkFunOTOS.Pose2D(-500, 860, Math.toRadians(-52)),
            sample3 = new SparkFunOTOS.Pose2D(-500, 860, Math.toRadians(-62)),
            humanReverse = new SparkFunOTOS.Pose2D(-530, 860, Math.toRadians(-130)),
            spitDetection = new SparkFunOTOS.Pose2D(-627, 430, Math.toRadians(-140)),
            humanTake = new SparkFunOTOS.Pose2D(60, 780 ,Math.toRadians(0));
    public static int samplesScored = 0;
    public static int type = 2;
    private static int tries = 0;

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
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen + 20);
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
                            Chassis.drive(0, 0.5, 0);
                            return true;
                        }
                    })
//                    .lineToAsync(Localizer.getCurrentPosition())
                    .addTask(new retractAsyncHelper())
                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.drive(0, 0, 0);
                            return true;
                        }
                    })
                    .waitSeconds(0.2)
//                    .addTask(new Sample.TakeSample(type, 0.5))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.startFollow();
                            if(tries >= 2){
                                r.skip();
                                return true;
                            }
                            if(s == null){
                                s = new Scheduler();
                                s.addTask(new Sample.TakeSample(type, 0.5))
                                        .addTask(new Task() {
                                            @Override
                                            public boolean Run() {
                                                tries ++;
                                                return true;
                                            }
                                        });
                            }
                            s.update();
                            if(s.done() && Storage.getStorageStatus() != GetPositionSample.getType(type)){
                                s = new Scheduler();
                                s.addTask(new Sample.TakeSample(type, 0.5))
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
                            Extendo.Extend(0);
                            return Extendo.getCurrentPosition() < 40;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.isStorageEmpty()){
                                r.clear();
                            }
                            return true;
                        }
                    })
                    .lineToAsync(spitDetection)
//                    .waitForTrajDone(90)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, spitDetection.h) < Math.toRadians(20);
                        }
                    })
                    .addTask(new SpitToHP(850, 0.8))
            ;
        }
        @Override
        public boolean Run() {
            r.update();
            return r.done();
        }
    }

    public static class ScoreSpecimen extends Task{
        private final Scheduler r;
        public ScoreSpecimen(double o){
            r = new Scheduler();

            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.close();
                            return true;
                        }
                    })
//                    .lineToAsync(new SparkFunOTOS.Pose2D(scoreSpecimen.x, scoreSpecimen.y + o, scoreSpecimen.h))
//                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(-600, scoreSpecimen.y - o, scoreSpecimen.h), new SparkFunOTOS.Pose2D(scoreSpecimen.x, scoreSpecimen.y - o, 0)))
                    .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(-650, scoreSpecimen.y, Math.toRadians(45)), scoreSpecimen))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = false;
                            Arm.setArmAngle(OutTakeLogic.ArmScoreSpecimen);
                            Extension.Extend(0);
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen);
//                            return Arm.getCurrentArmAngle() < 300;
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
//                            if(samplesScored == 0){
//                                return Localizer.getCurrentPosition().x < -710;
//                            }
                            if(Localizer.getCurrentPosition().y <= 0){
                                Chassis.asyncFollow = false;
//                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(scoreSpecimen.x, scoreSpecimen.y, 0));
                            }
                            if(Localizer.getCurrentPosition().x <= scoredLine - 150) Chassis.hProfile.setInstant(0);
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
                            Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 10);
                            Elevator.setTargetPosition(0);
                            Extension.Extend(OutTakeLogic.TakeSpecimenExtension);
                            return Localizer.getCurrentPosition().x >= -10;
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
                            Claw.close();
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
                            return Extendo.getCurrentPosition() > pos - 20 || Storage.hasTeamPice();
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

        Extendo.Extend(0);
        Extension.Extend(0);

        Chassis.setProfiles(3500, 3500, 7000, 7000, 1000, 1000);
        Chassis.setHeadingProfiles(10*Math.PI, 10*Math.PI, 100*Math.PI);
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
                        Extendo.Extend(300);
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-55);
                    }
                })
//                .waitForTrajDone(98)
//                .waitForSync()
                .addTask(new TakeSample(650))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(850, 1))

                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        Extendo.Extend(300);
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-75);
                    }
                })
                .addTask(new TakeSample(700))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(850, 1))

                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h > Math.toRadians(-100);
                    }
                })
                .addTask(new TakeSample(850))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(850, 1))
//                .lineToAsync(humanTake)
//                .lineToAsync(new SparkFunOTOS.Pose2D(humanTake.x - 120, humanTake.y + 50, 0))
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 150, humanTake.y, 0), humanTake))

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
                        Claw.openWide();
                        Extendo.Extend(0);
//                        Chassis.setProfiles(3000, 3000, 7000, 7000, 800, 800);
                        return true;
                    }
                })
//                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(100))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, 0), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(0))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, 0), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(0))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 50, 0), humanTake))
                .addTask(new SpecimenTake(tries < 3))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(tries > 3) requestOpModeStop();
                        return true;
                    }
                })
                .addTask(new ScoreSpecimen(0))

//                .lineToAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, 0), humanTake))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen(0))

//                .lineToLinearHeadingAsync(humanTake)
                .splineToAsync(Arrays.asList(new SparkFunOTOS.Pose2D(humanTake.x - 100, humanTake.y + 100, 0), humanTake))
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