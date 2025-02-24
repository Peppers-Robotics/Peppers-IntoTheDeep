package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

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
    public static double ElevatorScoreSample, ElevatorScoreSample1 = 100, ElevatorScoreSample2 = 870; // 700
    public static double ElevatorScoreSpecimen = 220;
    public static double ArmUpSample = 180, PivotUpSample = 0, ElevatorUp = 200;
    public static double ArmScoreSample = 240, PivotScoreSample = 0; // 220
    public static double ArmTakeSpecimen = 330, PivotTakeSpecimen = 0;
    public static double ArmScoreSpecimen = 110, PivotScoreSpecimen = 0;
    public static double ArmIdle = 5, PivotIdle = 0, ElevatorIdle = -69, DropDownTransfer = 0, ArmTransfer = 0;
    public static boolean save2 = false;
    public static double coeff = 2;
    public static double TakeSpecimenExtension = 0.32, TransferExtension = 0.35, ScoreSampleExtension = 1;
    private static SparkFunOTOS.Pose2D scoredSample, scoredSpecimen;
    public enum States{
        IDLE,
        IDLE_WITH_SAMPLE,
        IDLE_TAKE_SPECIMEN,
        IDLE_SCORE_SPECIMEN,
        IDLE_SCORE_SAMPLE,
    }
    public static States CurrentState = States.IDLE;
    public static Scheduler currentTask = new Scheduler();
    public static boolean Transfering = false;

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
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 1;
                                        return Elevator.getCurrentPosition() < 10;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 0.5;
                                        return Arm.getPrecentOfArmMotionCompleted() > 90;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Extension.Extend(TakeSpecimenExtension);
                                        return true; // TODO: add motionProfile
                                    }
                                })
                                ;
                    }

                        Controls.GrabSpecimen = false;
                        CurrentState = States.IDLE_TAKE_SPECIMEN;
                    }
                    if (Controls.Transfer) {

                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Transfering = true;
                                        ActiveIntake.powerOn();
                                        Extension.Extend(TransferExtension);
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.open();
                                        ActiveIntake.Block();
                                        Arm.setArmAngle(ArmTransfer);
                                        DropDown.setDown(DropDownTransfer);
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 1;
                                        Extendo.PowerOnToTransfer = true;
                                        Extendo.Extend(0);
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
                                        Elevator.PowerOnDownToTakeSample = false;
                                        Elevator.Disable = false;
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
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Extension.Extend(0);
                                        Transfering = false;
                                        return true;
                                    }
                                })
                        ;
                    }

                        CurrentState = States.IDLE_WITH_SAMPLE;
                        Controls.Transfer = false;
                    } else Transfering = false;
                    break;
                case IDLE_SCORE_SAMPLE:
                    Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * coeff);
                    if(save2){
                        ElevatorScoreSample2 = Elevator.getTargetPosition();
                    } else {
                        ElevatorScoreSample1 = Elevator.getTargetPosition();
                    }
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
                                            Extension.Retract();
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.2)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Localizer.Update();
                                            Arm.setArmAngle(ArmTransfer);
                                            if(Arm.getCurrentArmAngle() < 200) Claw.close();
                                            return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), scoredSample) > 80;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            if(Arm.getCurrentArmAngle() < 200) Claw.close();
//                                            Arm.setArmAngle(ArmTransfer);
                                            Elevator.setTargetPosition(ElevatorUp);
                                            return Arm.motionCompleted();
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorIdle);
                                            Elevator.PowerOnDownToTakeSample = true;
                                            Elevator.power = 1;
                                            Arm.setArmAngle(ArmIdle);
                                            return Elevator.getCurrentPosition() <= 10;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Claw.open();
                                            Elevator.PowerOnDownToTakeSample = false;
                                            return true;
                                        }
                                    })
                            ;
                        }

                        Controls.Grab = false;
                        CurrentState = States.IDLE;
                        Controls.ScoreLevel2 = false;
                        Controls.ScoreLevel1 = false;
                    }
//                    break;
                case IDLE_WITH_SAMPLE:
//                    Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * coeff);
                    if (Controls.ScoreLevel1) {

                        save2 = false;
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
                        Controls.Grab = false;
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
                                        Extendo.PowerOnToTransfer = false;
//                                        Extension.Extend(ScoreSampleExtension);
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.setArmAngle(ArmScoreSample);
                                        if(Arm.getCurrentArmAngle() > 180) Extension.Extend(0);
                                        Arm.setPivotAngle(PivotScoreSample);
                                        return Arm.motionCompleted();
                                    }
                                });
                    }
                        ElevatorScoreSample = ElevatorScoreSample2;
                        Controls.ScoreLevel2 = false;
                        Controls.Grab = false;
                        CurrentState = States.IDLE_SCORE_SAMPLE;
                        save2 = true;
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
                                            Elevator.Disable = false;
                                            Elevator.power = 1;
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
                                    .waitSeconds(0.2)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Controls.GrabSpecimen = false;
                                            Elevator.PowerOnDownToTakeSample = false;
                                            Elevator.Disable = false;
                                            Elevator.setTargetPosition(ElevatorScoreSpecimen);
                                            Arm.setArmAngle(ArmScoreSpecimen);
                                            if (Arm.getCurrentArmAngle() < 250)
                                                Arm.setPivotAngle(PivotScoreSpecimen);
                                            if(Arm.getCurrentArmAngle() < 120) Extension.Extend(100);
                                            return Arm.motionCompleted() && Elevator.ReachedTargetPosition();
                                        }
                                    })
                            ;
                        }
                        Controls.Grab = false;
                        CurrentState = States.IDLE_SCORE_SPECIMEN;
                        Controls.GrabSpecimen = false;
                    }
                    break;
                case IDLE_SCORE_SPECIMEN:
                    Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * coeff);
                    if (Controls.GrabSpecimen || Controls.gamepad1.wasPressed.square) {
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
                                            Extension.Extend(0);
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.2)
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
                                            Elevator.power = 0.8;
                                            CurrentState = States.IDLE_TAKE_SPECIMEN;
                                            Controls.GrabSpecimen = false;
                                            Controls.Grab = false;
                                            if(Arm.motionCompleted()){
                                                Extension.Extend(TakeSpecimenExtension);
                                            }
                                            return Arm.motionCompleted();
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
                                            return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), scoredSample) < 70 &&
                                                    Localizer.getCurrentPosition().x - scoredSpecimen.x < 0;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmTransfer);
                                            Elevator.setTargetPosition(ElevatorIdle);
                                            return Arm.motionCompleted();
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmIdle);
                                            return true;
                                        }
                                    })
                            ;
                        }
                        Controls.Grab = false;
                        Controls.Transfer = false;
                    }
                    break;
            }
        }
//        if(Controls.ScoreLevel2){
//            currentTask = new Scheduler();
//            {
//                currentTask
//                        .addTask(new Task() {
//                            @Override
//                            public boolean Run() {
//                                Transfering = true;
//                                ActiveIntake.powerOff();
//                                Elevator.setTargetPosition(ElevatorScoreSample);
////                                return Math.abs(Elevator.getCurrentPosition() - ElevatorScoreSample) < 300;
//                                return Elevator.getCurrentPosition() > ElevatorUp - 50;
//                            }
//                        })
//                        .addTask(new Task() {
//                            @Override
//                            public boolean Run() {
//                                Extendo.PowerOnToTransfer = false;
//                                return true;
//                            }
//                        })
//                        .addTask(new Task() {
//                            @Override
//                            public boolean Run() {
//                                Arm.setArmAngle(ArmScoreSample);
//                                Arm.setPivotAngle(PivotScoreSample);
//                                return true;
//                            }
//                        })
//                        .addTask(new Task() {
//                            @Override
//                            public boolean Run() {
////                                Arm.setArmAngle(ArmScoreSample);
////                                Arm.setPivotAngle(PivotScoreSample);
//                                if(Arm.getCurrentArmAngle() > 180){
//                                    Extension.Extend(ScoreSampleExtension);
//                                }
//                                Transfering = false;
//                                return Arm.motionCompleted();
//                            }
//                        });
//            }
//            ElevatorScoreSample = ElevatorScoreSample2;
//            Controls.ScoreLevel2 = false;
//            Controls.Grab = false;
//            CurrentState = States.IDLE_SCORE_SAMPLE;
//            save2 = true;
//        }
        if(Controls.Retract){
            currentTask.removeAllTasks();
            currentTask = new Scheduler();
            {
            currentTask
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            Extension.Retract();
//                            if(Elevator.getCurrentPosition() > ElevatorUp)
//                            Elevator.setTargetPosition(ElevatorUp + 100);
//                            if(Arm.getCurrentArmAngle() < 200) Claw.close();
                            return true;
                        }
                    })
                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Arm.setArmAngle(ArmTransfer);
                            return Arm.motionCompleted() && Arm.armProfile.getTargetPosition() == ArmTransfer;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.setTargetPosition(ElevatorIdle);
                            Elevator.PowerOnDownToTakeSample = true;
                            Elevator.power = 1;
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Controls.ScoreLevel2){
                                Elevator.PowerOnDownToTakeSample = false;
                                CurrentState = States.IDLE_WITH_SAMPLE;
                                currentTask.clear();
                                return true;
                            }
                            return Elevator.getCurrentPosition() < 5;
//                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.PowerOnDownToTakeSample = false;
                            Arm.setArmAngle(ArmIdle);
                            Claw.open();
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