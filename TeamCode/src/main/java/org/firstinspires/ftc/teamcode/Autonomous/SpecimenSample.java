package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "6 + 0")
@Disabled
@Config
public class SpecimenSample extends LinearOpMode {
    public static class retractAsyncHelper extends Task{
        private Scheduler s;
        public retractAsyncHelper(){
            s = new Scheduler();
            s
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.openWide();
                            Arm.setArmAngle(Arm.getCurrentArmAngle() + 15);
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
                    });
        }

        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
    public static double scoredLine = -700;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(-750, -280, Math.toRadians(20)),
            scoreSpecimen1 = new SparkFunOTOS.Pose2D(-750, -300, Math.toRadians(0)),
            sample1 = new SparkFunOTOS.Pose2D(-560, 570, Math.toRadians(-32)),
            sample2 = new SparkFunOTOS.Pose2D(-560, 570, Math.toRadians(-52)),
            sample3 = new SparkFunOTOS.Pose2D(-560, 600, Math.toRadians(-67)),
            humanReverse = new SparkFunOTOS.Pose2D(-530, 570, Math.toRadians(-150)),
            spitDetection = new SparkFunOTOS.Pose2D(-627, 130, Math.toRadians(-140)),
            humanTake = new SparkFunOTOS.Pose2D(40, 440 ,0),
            takeYellow = new SparkFunOTOS.Pose2D(-510, 80, Math.toRadians(-115)),
            basket = new SparkFunOTOS.Pose2D(-92, -1990, Math.toRadians(-60)),
            park = new SparkFunOTOS.Pose2D(2, 306, Math.toRadians(-90))
    ;
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
                    .waitSeconds(0.12)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = false;
                            Arm.setArmAngle(OutTakeLogic.ArmScoreSpecimen);
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen + 10);
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
                            return Localizer.getCurrentPosition().x <= -500; // TODO: change if needed
                        }
                    })
//                    .lineToAsync(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x , Localizer.getCurrentPosition().y, Math.toRadians(0)))
                    .lineTo(new SparkFunOTOS.Pose2D(-550, -300, Localizer.getCurrentPosition().h))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            retractFirst = true;
                            return true;
                        }
                    })
//                    .addTask(new retractAsyncHelper())
//                    .waitForSync()
//                    .waitForStill()
//                    .addTask(new Sample.TakeSample(type, 0.5))
                    .addTask(new Task() {
                        long time = -1;
                        @Override
                        public boolean Run() {
                            if(time == -1) time = System.currentTimeMillis();
                            if(tries >= 1){
                                r.clear();
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
                                                tries ++;
                                                return true;
                                            }
                                        });
                            }
//                            if((System.currentTimeMillis() - time) / 1000.f >= 5){
//                                tries = 10;
//                                return true;
//                            }
                            return s.done();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(0);
                            return Extendo.getCurrentPosition() < 100;
                        }
                    })
                    .lineToAsync(spitDetection)
                    .waitForTrajDone(90)
                    /*.addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, spitDetection.h) < Math.toRadians(30);
                        }
                    })*/
                    .addTask(new SpitToHP(850, 0.55))
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
        public ScoreSpecimen(){
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
                    .lineToAsync(new SparkFunOTOS.Pose2D(scoreSpecimen.x, scoreSpecimen.y, Math.toRadians(20)))
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
                            return Localizer.getCurrentPosition().x < scoredLine; // TODO: change if needed
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
                            if(scoredSecond){
                                Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 3);
                            } else {
                                Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                            }
                            Elevator.setTargetPosition(0);
                            Extension.Extend(OutTakeLogic.TakeSpecimenExtension);
                            return true;
                        }
                    })
                    .waitForTrajDone(50)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = powerLift;
                            Elevator.power = 0.7;
                            return true;
                        }
                    })
                    .waitForTrajDone(90)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Elevator.PowerOnDownToTakeSample = powerLift;
                            return Elevator.getCurrentPosition() < 50;
                        }
                    })
//                    .waitForTrajDone(powerLift ? 99.8 : 1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getCurrentPosition().x >= -6 /*&& Localizer.getAngleDifference(Localizer.getCurrentPosition().h, 0) <= Math.toRadians(1)*/ || !powerLift;
                        }
                    })
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
    public static boolean scoredSecond = false;
    public static class TakeSample extends Task{
        private final Scheduler r;
        public TakeSample(int pos){
            r = new Scheduler();
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Claw.closeAbit();
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
                                return Localizer.getCurrentPosition().h < Math.toRadians(-120);
                            }
                            return Localizer.getCurrentPosition().h < Math.toRadians(-110);
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
        Elevator.RESET = false;
        Elevator.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Scheduler auto = new Scheduler();
        scoredSecond = false;

        Elevator.setTargetPosition(0);
        Arm.setArmAngle(70);
        Claw.close();

        Extendo.Extend(0);
        Extension.Extend(0);

        Chassis.setProfiles(5000, 5000, 7000, 7000, 1000, 1000);
        Chassis.setHeadingProfiles(6*Math.PI, 4*Math.PI, 8*Math.PI);

        samplesScored = 0;
        tries = 0;

        Sample.camera = hardwareMap.get(Limelight3A.class, "camera");
        Sample.camera.start();
        Sample.camera.pipelineSwitch(0);
        Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        auto
//                .addTask(new ScoreSpecimen())
                .addTask(new ScoreSpecimen1())
                .lineToAsync(sample1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(200);
                        ActiveIntake.powerOn();
                        Arm.setArmAngle(Arm.getCurrentArmAngle() + 50);
                        Claw.close();
                        DropDown.setDown(1);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-65);
                    }
                })
//                .waitForTrajDone(98)
//                .waitForSync()
                .addTask(new TakeSample(350))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(300, 1))

                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(100);
                        ActiveIntake.powerOn();
                        Arm.setArmAngle(270);
                        DropDown.setDown(1);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-70);
                    }
                })
                .addTask(new TakeSample(600))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Chassis.setProfiles(1e8, 1e8, 5e8, 5e8, 2e6, 2e6);
                        return true;
                    }
                })
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(300, 1))

                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        // if not work uncomment
                        Claw.openWide();
                        scoredSecond = true;
                        Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 30);
                        return Localizer.getCurrentPosition().h > Math.toRadians(-90);
                    }
                })
                .addTask(new TakeSample(850))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(300, 1))
                .lineToAsync(new SparkFunOTOS.Pose2D(-100, humanTake.y, humanTake.h))

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
                        Claw.openWide();
                        Extendo.Extend(0);
//                        Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                        return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, 0) < Math.toRadians(10);
                    }
                })
//                .addTask(new SpecimenTake(false))
                .lineToLinearHeadingAsync(new SparkFunOTOS.Pose2D(40, humanTake.y + 20, 0))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        scoredSecond = false;
                        return true;
                    }
                })

                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(tries <= 2))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(tries > 2) requestOpModeStop();
                        return true;
                    }
                })
                .addTask(new ScoreSpecimen())

                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToAsync(humanTake)
                .addTask(new SpecimenTake(false))
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
        retractA = new Scheduler();
        doTransfer = new Scheduler();
        retractA.addTask(new retractAsyncHelper());
        doTransfer.addTask(new Sample.Transfer());

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
            if(retractFirst){
                retractA.update();
                if(retractA.done()) retractFirst = false;
            }
            if(transfer){
                 doTransfer.update();
                 if(doTransfer.done()) transfer = false;
            }

            Elevator.update();
            Extendo.update();
            Arm.update();
            Localizer.Update();
            Chassis.Update();
            Robot.clearCache();
        }

    }
    private static boolean retractFirst = false, transfer = false;
    private static Scheduler retractA, doTransfer;
}