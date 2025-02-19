package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Autonomous(name = "5 + 0")
public class Specimen extends LinearOpMode {
    public static double scoredLine = 0;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(0, 0, 0),
        sample1 = new SparkFunOTOS.Pose2D(0, 0, 0),
        sample2 = new SparkFunOTOS.Pose2D(0, 0, 0),
        sample3 = new SparkFunOTOS.Pose2D(0, 0, 0),
        humanReverse = new SparkFunOTOS.Pose2D(0, 0, 0),
        humanPreTake = new SparkFunOTOS.Pose2D(0, 0, 0),
        humanTake = new SparkFunOTOS.Pose2D(0, 0 ,0);

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
                            Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSpecimen);
                            return Arm.getCurrentArmAngle() < 300;
                        }
                    })
                    .lineToAsync(scoreSpecimen)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getCurrentPosition().x > scoredLine; // TODO: change if needed
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
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
                            Claw.open();
                            Arm.setArmAngle(OutTakeLogic.ArmTakeSpecimen);
                            Elevator.setTargetPosition(0);
                            return true;
                        }
                    })
                    .waitForTrajDone(90)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = powerLift;
                            Elevator.power = 1;
                            return Elevator.getCurrentPosition() < 50;
                        }
                    })
                    .waitForTrajDone(powerLift ? 90 : 0)
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.close();
                            return true;
                        }
                    })
                    .waitSeconds(0.15)
            ;
        }

        @Override
        public boolean Run() {
            r.update();
            return r.done();
        }
    }
    public static class TakeSample extends Task{
        private Scheduler r;
        public TakeSample(int pos){
            r = new Scheduler();
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            DropDown.setDown(1);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > pos - 20;
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
        private Scheduler r;

        public SpitToHP(int pos){
            r
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(pos);
                            return Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) < 20;
                        }
                    })
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            ActiveIntake.Reverse(1);
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
        Scheduler auto = new Scheduler();

        auto
                .addTask(new ScoreSpecimen())

                .lineToLinearHeadingAsync(sample1)
                .waitForTrajDone(75)
                .addTask(new TakeSample(400))
                .lineToLinearHeadingAsync(humanReverse)
                .addTask(new SpecimenTake(false))
                .addTask(new SpitToHP(300))

                .lineToLinearHeadingAsync(sample2)
                .waitForTrajDone(80)
                .addTask(new TakeSample(500))
                .lineToLinearHeadingAsync(humanReverse)
                .addTask(new SpitToHP(300))

                .lineToLinearHeadingAsync(sample3)
                .waitForTrajDone(80)
                .addTask(new TakeSample(800))
                .lineToLinearHeadingAsync(humanReverse)
                .addTask(new SpitToHP(300))

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
                        return true;
                    }
                })
                .lineToLinearHeadingAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToLinearHeadingAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToLinearHeadingAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                .lineToLinearHeadingAsync(humanTake)
                .addTask(new SpecimenTake(true))
                .addTask(new ScoreSpecimen())

                ;

        while(opModeInInit()){
            Elevator.update();
            Extendo.update();
            Arm.update();
            Robot.clearCache();
            DropDown.setDown(0);
        }

        while (opModeIsActive()) {
            auto.update();

            Elevator.update();
            Extendo.update();
            Arm.update();
            Robot.clearCache();
        }

    }
}
