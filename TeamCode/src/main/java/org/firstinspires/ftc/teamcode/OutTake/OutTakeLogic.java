package org.firstinspires.ftc.teamcode.OutTake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
public class OutTakeLogic {
    public static double ElevatorScoreSample = 780, ElevatorScoreSample1 = 230, ElevatorScoreSample2 = 780; // 700
    public static double ElevatorScoreSpecimen = 310;
    public static double ArmUpSample = 180, PivotUpSample = 0, ElevatorUp = 200;
    public static double ArmScoreSample = 235, PivotScoreSample = 0; // 220
    public static double ArmTakeSpecimen = 325, PivotTakeSpecimen = 0;
    public static double ArmScoreSpecimen = 95, PivotScoreSpecimen = 0;
    public static double ArmIdle = -10, PivotIdle = 0, ElevatorIdle = -10, DropDownTransfer = 0, ArmTransfer = -9;
    public static boolean save2 = false;
    public static double coeff = 5;
    public static double TakeSpecimenExtension = 0.2, TransferExtension = 0.24, ScoreSampleExtension = 0.5, takeSpecimenPower = 0.2;
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
                                        Claw.openWide();
                                        Elevator.setTargetPosition(100);
                                        if (Elevator.getCurrentPosition() > 90) {
                                            Arm.setArmAngle(ArmTakeSpecimen - 5);
                                        }
                                        return Arm.getCurrentArmAngle() > 90;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.PowerOnDownToTakeSample = true;
                                        Elevator.power = 0.5;
                                        return Elevator.getCurrentPosition() < 10;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.PowerOnDownToTakeSample = false;
                                        Elevator.setTargetPosition(ElevatorIdle);
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
                                        Arm.ShouldDoOffset = false;
                                        Transfering = true;
                                        ActiveIntake.powerOn();
                                        Extension.Extend(TransferExtension);
//                                        IntakeLogic.wasDriverActivated = true;
//                                        Arm.setArmAngle(ArmTransControlsfer);
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
                                        Elevator.power = 0.8;
                                        Extendo.PowerOnToTransfer = true;
                                        Extendo.Extend(25);
                                        return true;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        return Elevator.getCurrentPosition() < 30 && Extendo.getCurrentPosition() < 50;
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
                                        Arm.ShouldDoOffset = false;
//                                        IntakeLogic.wasDriverActivated = false;
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
                                            return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), scoredSample) > 130;
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

//                        Controls.Grab = false;
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
                                            Elevator.setTargetPosition(ElevatorScoreSample);
                                            return Elevator.getCurrentPosition() >= ElevatorUp - 100;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Arm.setArmAngle(ArmScoreSample);
                                            Arm.setPivotAngle(PivotScoreSample);
                                            return Arm.getCurrentArmAngle() > ArmScoreSample - 5 && Elevator.getCurrentPosition() > ElevatorScoreSample - 200;
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
                                        return /*Arm.motionCompleted() && */Elevator.getCurrentPosition() > ElevatorScoreSample - 400;
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
                    if(Storage.hasTeamPice() && Controls.Throw){
                        Controls.Throw = false;
                        currentTask = new Scheduler();
                        currentTask
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.setArmAngle(ArmTransfer);
                                        Elevator.setTargetPosition(100);
                                        Claw.open();
                                        return Arm.getCurrentArmAngle() < 20;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Elevator.setTargetPosition(0);
                                        return true;
                                    }
                                }) .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.ShouldDoOffset = false;
                                        Transfering = true;
                                        ActiveIntake.powerOn();
                                        Extension.Extend(TransferExtension);
//                                        IntakeLogic.wasDriverActivated = true;
//                                        Arm.setArmAngle(ArmTransControlsfer);
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
                                        Elevator.power = 0.8;
                                        Extendo.PowerOnToTransfer = true;
                                        Extendo.Extend(25);
                                        return true;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        return Elevator.getCurrentPosition() < 30 && Extendo.getCurrentPosition() < 50;
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
                                        Extension.Retract();
                                        Transfering = true;
                                        Arm.ShouldDoOffset = false;
                                        return true;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Transfering = false;
                                        Elevator.setTargetPosition(200);
                                        return Elevator.getCurrentPosition() > 150;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Arm.setArmAngle(ArmTakeSpecimen - 50);
                                        Elevator.setTargetPosition(-600);
                                        return Arm.getCurrentArmAngle() > 100;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        return Controls.Grab;
                                    }
                                })
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.openWide();
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)
                                .addTask(new Task() {
                                    @Override
                                    public boolean Run() {
                                        Claw.openWide();
                                        Arm.setArmAngle(ArmTakeSpecimen);
                                        return true;
                                    }
                                })
                        ;

                    }
                    if (Controls.Grab) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Chassis.PuttingSpecimens = true;
                                            Chassis.DoingSpecimens = true;
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
                                            return Elevator.getCurrentPosition() < 20;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.PowerOnDownToTakeSample = false;
                                            Elevator.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            Elevator.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            return true;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Chassis.DoingSpecimens = true;
                                            Controls.GrabSpecimen = false;

                                            Elevator.Disable = false;
                                            Elevator.setTargetPosition(ElevatorScoreSpecimen);
                                            Arm.setArmAngle(ArmScoreSpecimen);
                                            if (Arm.getCurrentArmAngle() < 250)
                                                Arm.setPivotAngle(PivotScoreSpecimen);
                                            if(Arm.getCurrentArmAngle() < 120) Extension.Extend(0.2);
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
                    if(Controls.gamepad2.gamepad.right_bumper)
                        Extension.Extend(1);
                    else
                        Extension.Extend(0.2);
                    if (Controls.GrabSpecimen || Controls.gamepad1.wasPressed.square) {
                        currentTask = new Scheduler();
                        {
                            currentTask
                                    .addTask(new Task() {

                                        @Override
                                        public boolean Run() {
                                            Chassis.PuttingSpecimens = false;
                                            Claw.openWide();
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
                                            return Elevator.getCurrentPosition() < 20;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            Elevator.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            Elevator.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            return true;
                                        }
                                    })
                                    .addTask(new Task() {
                                        @Override
                                        public boolean Run() {
                                            //Elevator.PowerOnDownToTakeSample = true;
                                            //Elevator.power = takeSpecimenPower;
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
                                            return Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), scoredSample) < 130 &&
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
                            Chassis.DoingSpecimens = false;
                            if(Elevator.getCurrentPosition() < 30)
                                IntakeLogic.IgnoreUntilNext = true;
                            Claw.open();
                            Extension.Retract();
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
                            return Elevator.getCurrentPosition() < 40;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Elevator.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            Elevator.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            return true;
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
            Controls.Transfer=false;
            Controls.RetractExtendo = false;

        }
        Robot.telemetry.addData("state", CurrentState.toString());
        currentTask.update();
    }

}