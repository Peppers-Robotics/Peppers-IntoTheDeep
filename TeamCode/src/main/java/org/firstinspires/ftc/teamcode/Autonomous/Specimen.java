package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class Specimen extends LinearOpMode {
    public static double scoredLine = -700;
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(-713, -87, Math.toRadians(20)),
        sample1 = new SparkFunOTOS.Pose2D(-627, 40, Math.toRadians(292)),
        sample2 = new SparkFunOTOS.Pose2D(-530, 560, Math.toRadians(308)),
        sample3 = new SparkFunOTOS.Pose2D(-530, 560, Math.toRadians(297)),
        humanReverse = new SparkFunOTOS.Pose2D(-530, 560, Math.toRadians(202)),
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
                            return Localizer.getCurrentPosition().x < scoredLine; // TODO: change if needed
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
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(OutTakeLogic.ArmIdle);
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
                    .waitForTrajDone(powerLift ? 90 : 1)
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
        private final Scheduler r;
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
        private final Scheduler r;

        public SpitToHP(int pos){
            r = new Scheduler();
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
        Elevator.setTargetPosition(0);
        Arm.setArmAngle(70);
        Claw.close();

        auto
                .addTask(new ScoreSpecimen())

                .lineToAsync(sample1)
                .waitForTrajDone(96)
                .addTask(new TakeSample(400))
                .lineToLinearHeadingAsync(humanReverse)
                .addTask(new SpecimenTake(false))
                .addTask(new SpitToHP(300))

                .lineToAsync(sample2)
                .waitForTrajDone(100)
                .addTask(new TakeSample(500))
                .lineToLinearHeadingAsync(humanReverse)
                .addTask(new SpitToHP(300))

                .lineToAsync(sample3)
                .waitForTrajDone(100)
                .addTask(new TakeSample(800))
                .lineToAsync(humanReverse)
                .addTask(new SpitToHP(300))

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
                        requestOpModeStop();
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
            Localizer.Update();
            Chassis.Update();
            Robot.clearCache();
        }

    }
}
