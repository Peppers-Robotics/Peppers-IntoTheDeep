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
                            Claw.closeAbit();
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
    public static double scoredLine = -600;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(-800, -280, Math.toRadians(20)),
        scoreSpecimen1 = new SparkFunOTOS.Pose2D(-750, -570, Math.toRadians(0)),
        sample1 = new SparkFunOTOS.Pose2D(-560, 570, Math.toRadians(-30)),
        sample2 = new SparkFunOTOS.Pose2D(-560, 570, Math.toRadians(-53)),
        sample3 = new SparkFunOTOS.Pose2D(-560, 600, Math.toRadians(-62)),
        humanReverse = new SparkFunOTOS.Pose2D(-560, 570, Math.toRadians(-150)),
        spitDetection = new SparkFunOTOS.Pose2D(-627, 130, Math.toRadians(-140)),
        humanTake = new SparkFunOTOS.Pose2D(60, 450 , 0);
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
                            if(Localizer.getCurrentPosition().x <= -500)
                                Extendo.Extend(500);
                            DropDown.setDown(0);
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            return Localizer.getCurrentPosition().x <= -600; // TODO: change if needed
                        }
                    })
                    .addTask(new retractAsyncHelper())
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Extendo.getCurrentPosition() > 480;
                        }
                    })
                    .addTask(new Task() {
                        long time = -1;
                        @Override
                        public boolean Run() {
                            if(time == -1) time = System.currentTimeMillis();
                            DropDown.setDown(1);
                            return Storage.hasTeamPice() || (System.currentTimeMillis() - time) / 1000.f >= 0.3;
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
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Reverse(0.4);
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
                    .addTask(new SpitToHP(850, 0.6))
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
                            Claw.closeAbit();
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
                            Claw.closeAbit();
                            if(scoredSecond){
                                Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen - 3);
                            } else {
                                Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                            }
                            Elevator.setTargetPosition(0);
                            Extension.Extend(OutTakeLogic.TakeSpecimenExtension);
                            return Elevator.getCurrentPosition() <= 30;
                        }
                    })
                    .waitForTrajDone(50)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = powerLift;
                            Elevator.power = 0.5;
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
                            return Localizer.getCurrentPosition().x >= -1;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.close();
                            return true;
                        }
                    })
                    .waitSeconds(0.02)
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
                            DropDown.setDown(0);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > pos - 20 || Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        private long time = -1;
                        @Override
                        public boolean Run() {
                            DropDown.setDown(1);
                            if(time == -1){
                                time = System.currentTimeMillis();
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.1){
                                Extendo.Extend(pos + 100);
                            }
                            return Storage.hasTeamPice() || (double) (System.currentTimeMillis() - time) / 1000.f >= 0.3;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Extendo.Extend(300);
                            ActiveIntake.Block();
                            return true;
                        }
                    })
//                    .waitSeconds(0.05)
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

        Chassis.setProfiles(5000, 5000, 8000, 8000, 800, 800);
        Chassis.setHeadingProfiles(6*Math.PI, 4*Math.PI, 8*Math.PI);

        samplesScored = 0;
        tries = 0;

        Sample.camera = hardwareMap.get(Limelight3A.class, "camera");
        Sample.camera.start();
        Sample.camera.pipelineSwitch(0);
        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-70);
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
//                        Chassis.setHeadingProfiles(8*Math.PI, 4 * Math.PI, 10* Math.PI);
                        return Localizer.getCurrentPosition().h >= Math.toRadians(-60);
                    }
                })
                .addTask(new TakeSample(600))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(300, 1))

                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        // if not work uncomment
                        Claw.closeAbit();
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
                        Claw.closeAbit();
                        Extendo.Extend(0);
//                        Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                        return Localizer.getAngleDifference(Localizer.getCurrentPosition().h, 0) < Math.toRadians(10);
                    }
                })
//                .addTask(new SpecimenTake(false))
                .waitForTrajDone(80)
                .lineToAsync(new SparkFunOTOS.Pose2D(40, humanTake.y + 20, 0))
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .addTask(new TakeSample(500))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        DropDown.setDown(0);
                        Extendo.Extend(0);
                        return Extendo.getCurrentPosition() < 50;
                    }
                })
                .lineToAsync(new SparkFunOTOS.Pose2D(spitDetection.x, spitDetection.y, spitDetection.h))
//                .waitForSync()
                .waitForTrajDone(95)
                .addTask(new SpitToHP(850, 0.5))
//                .waitForSync()

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        Elevator.power = 1;
                        Elevator.PowerOnDownToTakeSample = true;
                        scoredSecond = false;
                        return true;
                    }
                })

//                .lineToAsync(humanTake)
                .lineToAsync(new SparkFunOTOS.Pose2D(40, humanTake.y + 60, 0))
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

                .lineToLinearHeadingAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())


                .lineToLinearHeadingAsync(humanTake)
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
            Elevator.setTargetPosition(0);
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
