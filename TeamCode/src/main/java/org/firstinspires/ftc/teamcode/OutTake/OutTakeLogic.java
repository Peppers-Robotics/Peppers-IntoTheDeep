package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
public class OutTakeLogic {
    public static double ElevatorScoreSample, ElevatorScoreSample1 = 500, ElevatorScoreSample2 = 930;
    public static double ElevatorScoreSpecimen = 600;
    public static double ArmUpSample = 180, PivotUpSample = 0, ElevatorUp = 200;
    public static double ArmScoreSample = 260, PivotScoreSample = 0;
    public static double ArmTakeSpecimen = 350, PivotTakeSpecimen = 0;
    public static double ArmScoreSpecimen = 110, PivotScoreSpecimen = 0;
    public static double ArmIdle = 25, PivotIdle = 0, ElevatorIdle = -69, DropDownTransfer = 0, ArmTransfer = 20;
    private static Pose2D scoredSample = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0), scoredSpecimen;
    public enum States{
        IDLE,
        IDLE_WITH_SAMPLE,
        IDLE_TAKE_SPECIMEN,
        IDLE_SCORE_SPECIMEN,
        IDLE_SCORE_SAMPLE,
    }
    public static States CurrentState = States.IDLE;
    private static Scheduler currentTask = new Scheduler();

    public static void update(){
        if(currentTask.done()) {

            switch (CurrentState) {
                case IDLE:
                    if (Controls.GrabSpecimen) {
                        Controls.Transfer = false;
                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.setTargetPosition(ElevatorUp);
                                        if (Elevator.getCurrentPosition() > ElevatorUp - 80) {
                                            Arm.setArmAngle(ArmTakeSpecimen);
                                        }
                                        return Arm.getCurrentArmAngle() > 90;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.setTargetPosition(ElevatorIdle);
                                        return Elevator.getCurrentPosition() < 10;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 0.5;
                                        return true;
                                    }
                                })
                                ;
                    }

                        Controls.GrabSpecimen = false;
                        CurrentState = States.IDLE_TAKE_SPECIMEN;
                    }
                    if (Storage.hasTeamPice() && Controls.Transfer) {

                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.open();
                                        ActiveIntake.Block();
                                        Arm.setArmAngle(ArmTransfer);
                                        DropDown.setDown(DropDownTransfer);
                                        if (!ActiveIntake.isOff()) {
                                            return false;
                                        }
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 1;
                                        Extendo.PowerOnToTransfer = true;
                                        ActiveIntake.powerOn();
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.close();
                                        return true;
                                    }
                                })
                                .waitSeconds(0.2)
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        ActiveIntake.Unblock();
                                        Elevator.PowerOnDownToTakeSample = false;
                                        Extendo.PowerOnToTransfer = false;
                                        ActiveIntake.powerOff();
                                        Elevator.setTargetPosition(ElevatorUp);
                                        if (Elevator.getCurrentPosition() > ElevatorUp - 80) {
                                            Arm.setArmAngle(ArmUpSample);
                                            Arm.setPivotAngle(PivotUpSample);
                                        }
                                        return Arm.getCurrentArmAngle() >= 100;
                                    }
                                })
                        ;
                    }

                        CurrentState = States.IDLE_WITH_SAMPLE;
                        Controls.Transfer = false;
                    }
                case IDLE_WITH_SAMPLE:
                    if (Controls.ScoreLevel1) {

                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorScoreSample);
                                            return Math.abs(Elevator.getCurrentPosition() - ElevatorScoreSample) < 120;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmScoreSample);
                                            Arm.setPivotAngle(PivotScoreSample);
                                            return Arm.motionCompleted();
                                        }
                                    });
                        }
                        ElevatorScoreSample = ElevatorScoreSample1;
                        Controls.ScoreLevel1 = false;
                        CurrentState = States.IDLE_SCORE_SAMPLE;
                    }
                    if (Controls.ScoreLevel2) {

                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.setTargetPosition(ElevatorScoreSample);
                                        return Math.abs(Elevator.getCurrentPosition() - ElevatorScoreSample) < 300;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.setArmAngle(ArmScoreSample);
                                        Arm.setPivotAngle(PivotScoreSample);
                                        return Arm.motionCompleted();
                                    }
                                });
                    }
                        ElevatorScoreSample = ElevatorScoreSample2;
                        Controls.ScoreLevel2 = false;
                        CurrentState = States.IDLE_SCORE_SAMPLE;
                    }
                    break;
                case IDLE_SCORE_SAMPLE:
                    if (Controls.Grab) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Localizer.Update();
                                            scoredSample = Localizer.getCurrentPosition();
                                            Claw.open();
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.3)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Localizer.Update();
                                            Arm.setArmAngle(ArmTransfer);
                                            return Localizer.getDistanceFromTwoPoints(DistanceUnit.INCH, Localizer.getCurrentPosition(), scoredSample) > 3;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
//                                            Arm.setArmAngle(ArmTransfer);
                                            Elevator.setTargetPosition(ElevatorUp);
                                            return Arm.motionCompleted();
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorIdle);
                                            Arm.setArmAngle(ArmIdle);
                                            return true;
                                        }
                                    });
                        }

                        Controls.Grab = false;
                        CurrentState = States.IDLE;
                    }
                    break;
                case IDLE_TAKE_SPECIMEN:
                    if (Controls.Grab) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.PowerOnDownToTakeSample = true;
                                            return true;
                                        }
                                    })
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
                                            Elevator.Disable = false;
                                            Elevator.setTargetPosition(ElevatorScoreSpecimen);
                                            Arm.setArmAngle(ArmScoreSpecimen);
                                            if (Arm.getCurrentArmAngle() < 250)
                                                Arm.setPivotAngle(PivotScoreSpecimen);
                                            return Arm.motionCompleted() && Elevator.ReachedTargetPosition();
                                        }
                                    })
                            ;
                        }
                        Controls.Grab = false;
                        CurrentState = States.IDLE_SCORE_SPECIMEN;
                    }
                    break;
                case IDLE_SCORE_SPECIMEN:
                    if (Controls.GrabSpecimen) {
                        currentTask = new Scheduler();
                        {
                            currentTask
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
                                            Elevator.setTargetPosition(ElevatorUp);
                                            Arm.setArmAngle(ArmTakeSpecimen);
                                            return Elevator.getCurrentPosition() < ElevatorUp + 50;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorIdle);
                                            return Elevator.getCurrentPosition() < 10;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.PowerOnDownToTakeSample = true;
                                            Elevator.power = 0.5;
                                            CurrentState = States.IDLE_TAKE_SPECIMEN;
                                            return true;
                                        }
                                    })
                            ;
                        }
                        Controls.GrabSpecimen = false;
                    }
                    if (Controls.Grab) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Claw.open();
                                            Localizer.Update();
                                            scoredSpecimen = Localizer.getCurrentPosition();
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.2)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Localizer.Update();
                                            return Localizer.getDistanceFromTwoPoints(DistanceUnit.INCH, Localizer.getCurrentPosition(), scoredSpecimen) > 3;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmTransfer);
                                            Elevator.setTargetPosition(ElevatorUp);
                                            return Arm.motionCompleted();
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorIdle);
                                            Arm.setArmAngle(ArmIdle);
                                            return true;
                                        }
                                    })
                            ;
                        }
                        Controls.Grab = false;
                    }
                    break;
            }
        }
        if(Controls.Retract){
            currentTask.removeAllTasks();
            currentTask = new Scheduler();
            {
            currentTask
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            if(Elevator.getCurrentPosition() > ElevatorUp)
                                Arm.setArmAngle(ArmTransfer);
                            Elevator.setTargetPosition(ElevatorUp + 100);
                            return Arm.motionCompleted() && Arm.armProfile.getTargetPosition() == ArmTransfer;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.Disable = true;
                            Elevator.motor.setPower(-1);
                            Elevator.setTargetPosition(0);
                            return Elevator.getCurrentPosition() < 5;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.motor.setPower(0);
                            Elevator.Disable = false;
                            Arm.setArmAngle(ArmIdle);
                            return true;
                        }
                    })
            ;
            }
            CurrentState = States.IDLE;
            Controls.Retract = false;
        }

        currentTask.update();
    }

}