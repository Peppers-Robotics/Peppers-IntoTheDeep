package org.firstinspires.ftc.teamcode.OutTake;
import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Sample;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
public class OutTakeLogic {
    public static double ElevatorScoreSample = 780, ElevatorScoreSample1 = 200, ElevatorScoreSample2 = 800; // 700
    public static double ElevatorScoreSpecimen = 310;
    public static double ArmUpSample = 180, PivotUpSample = 0, ElevatorUp = 200;
    public static double ArmScoreSample = 235, PivotScoreSample = 0; // 220
    public static double ArmTakeSpecimen = 335, PivotTakeSpecimen = 0;
    public static double ArmScoreSpecimen = 95, PivotScoreSpecimen = 0;
    public static double ArmIdle = -10, PivotIdle = 0, ElevatorIdle = -69, DropDownTransfer = 0, ArmTransfer = -10;
    public static boolean save2 = false;
    public static double coeff = 5;
    public static double TakeSpecimenExtension = 0.31, TransferExtension = 0.26, ScoreSampleExtension = 0.5, takeSpecimenPower = 0.3;
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
//                    DropDown.setDown(0);
                    if (Controls.GrabSpecimen) {
                        Controls.Transfer = false;
                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.ShouldDoOffset = true;
                                        Claw.closeAbit();
                                        Elevator.setTargetPosition(ElevatorUp);
                                        if (Elevator.getCurrentPosition() > ElevatorUp - 80) {
                                            Arm.setArmAngle(ArmTakeSpecimen - 5);
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
                                        Elevator.power = takeSpecimenPower;
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
                    if (Controls.Transfer /*&& Storage.hasTeamPice()*/) {

                        currentTask = new Scheduler();
                        {
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.ShouldDoOffset = true;
                                        Transfering = true;
                                        ActiveIntake.powerOn();
                                        Extension.Extend(TransferExtension);
//                                        Arm.setArmAngle(ArmTransfer);
                                        return true;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.open();
                                        ActiveIntake.Block();
//                                        Arm.setArmAngle(ArmTransfer);
                                        DropDown.setDown(DropDownTransfer);
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 1;
                                        Extendo.PowerOnToTransfer = true;
                                        Extendo.Extend(0);
                                        return true;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        return Elevator.getCurrentPosition() < 10 && Extendo.getCurrentPosition() < 10;
                                    }
                                })
//                                .waitSeconds(0.05)
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
                                        Elevator.Disable = false;
                                        Extendo.PowerOnToTransfer = false;
                                        ActiveIntake.powerOff();
//                                        Elevator.setTargetPosition(ElevatorScoreSample);
//                                        if (Elevator.getCurrentPosition() > ElevatorUp - 80) {
//                                            Arm.setArmAngle(ArmUpSample);
//                                            Arm.setPivotAngle(PivotUpSample);
//                                        }
//                                        return Arm.getCurrentArmAngle() >= 100;
                                        Extension.Retract();
//                                        DropDown.setDown(0.6);
                                        Transfering = true;
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)

                        ;
                    }

                        CurrentState = States.IDLE_WITH_SAMPLE;
                    } else Transfering = false;
                    if (Controls.ScoreLevel2 && Storage.isStorageEmpty()){
                        Transfering = false;
                        currentTask = new Scheduler();

                        Elevator.setTargetPosition(ElevatorScoreSample2);
                        Controls.ScoreLevel2 = false;
                    }
                    break;
                case IDLE_SCORE_SAMPLE:
                    Transfering = false;
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
                                            Arm.ShouldDoOffset = false;
                                            Localizer.Update();
                                            scoredSample = Localizer.getCurrentPosition();
                                            Claw.open();
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.1)
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
//                                            Elevator.setTargetPosition(0);
//                                            Elevator.PowerOnDownToTakeSample = true;
//                                            Elevator.power = 1;
                                            return Arm.getCurrentArmAngle() < 100;
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
                        Controls.RetractExtendo = false;
                        Controls.Transfer = false;
                        CurrentState = States.IDLE;
                        Controls.ScoreLevel2 = false;
                        Controls.ScoreLevel1 = false;
                    }
//                    break;
                case IDLE_WITH_SAMPLE:
                    Transfering = false;
//                    Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * coeff);
                    if (Controls.ScoreLevel1) {

                        save2 = false;
                        ElevatorScoreSample = ElevatorScoreSample1;
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.ShouldDoOffset = false;
                                            Elevator.setTargetPosition(ElevatorScoreSample);
                                            return Elevator.getCurrentPosition() >= ElevatorUp - 100;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmScoreSample);
                                            Arm.setPivotAngle(PivotScoreSample);
                                            return Arm.motionCompleted() && Elevator.getCurrentPosition() > ElevatorScoreSample - 200;
                                        }
                                    });
                        }
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
                                        Elevator.PowerOnDownToTakeSample = false;
                                        Elevator.setTargetPosition(ElevatorScoreSample);
                                        return Elevator.getCurrentPosition() >= ElevatorUp - 100;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.setArmAngle(ArmScoreSample);
                                        Arm.setPivotAngle(PivotScoreSample);
                                        return /*Arm.motionCompleted() && */Elevator.getCurrentPosition() > ElevatorScoreSample - 200;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        return true;
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
                                    .waitSeconds(0.08)
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
                                            if(Arm.getCurrentArmAngle() < 120) Extension.Extend(1);
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
//                    Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * coeff);
                    if(Controls.gamepad2.wasPressed.right_bumper){
                        if(Extension.getPrecent() >= 0.2) Extension.Extend(0);
                        else Extendo.Extend(1);
                    }
                    if (Controls.GrabSpecimen || Controls.gamepad1.wasPressed.square) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Claw.closeAbit();
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.1)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(170);
                                            Extension.Extend(0);
                                            return true;
                                        }
                                    })
                                    .waitSeconds(0.2)
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.setTargetPosition(ElevatorUp);
                                            Arm.setArmAngle(ArmTakeSpecimen - 5);
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
                                            Elevator.power = takeSpecimenPower;
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
                        Controls.RetractExtendo = false;
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

            Robot.telemetry.addLine("RETRACT !$#%@$^%&^%*&%^$%#$");
            currentTask.removeAllTasks();
            currentTask = new Scheduler();
            Transfering = false;
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
                            return Arm.motionCompleted() && Arm.getArmTargetPosition() == ArmTransfer;
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
            Controls.Transfer = false;
            Controls.RetractExtendo = false;
        }

        currentTask.update();
    }

}