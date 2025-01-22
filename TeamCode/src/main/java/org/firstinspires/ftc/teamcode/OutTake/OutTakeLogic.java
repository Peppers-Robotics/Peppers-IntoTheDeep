package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Config
public class OutTakeLogic {
    public static double ElevatorScoreSample, ElevatorScoreSample1 = 500, ElevatorScoreSample2 = 980;
    public static double ElevatorScoreSpecimen;
    public static double ArmUpSample = 180, PivotUpSample = 0, ElevatorUp = 200;
    public static double ArmScoreSample = 180, PivotScoreSample = 0;
    public static double ArmTakeSpecimen = 330, PivotTakeSpecimen = -20;
    public static double ArmScoreSpecimen = 110, PivotScoreSpecimen = 185;
    public static double ArmIdle = 0, PivotIdle = 185, ElevatorIdle = 0;
    public enum States{
        IDLE,
        IDLE_WITH_SAMPLE,
        IDLE_TAKE_SPECIMEN,
        IDLE_SCORE_SPECIMEN,
        IDLE_SCORE_SAMPLE,
    }
    public static States CurrentState = States.IDLE;
    private final static Scheduler transfer, toScoreSample, ScoreSample, Retract, ToTakeSpecimen, TransferSpecimen, ScoreSpecimenReTake;
    private static Scheduler currentTask;
    static{
        transfer = new Scheduler();
        toScoreSample = new Scheduler();
        ScoreSample = new Scheduler();
        ScoreSpecimenReTake = new Scheduler();
        TransferSpecimen = new Scheduler();
        ToTakeSpecimen = new Scheduler();
        Retract = new Scheduler();

        Retract.addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(Elevator.getCurrentPosition() > ElevatorUp)
                            Arm.setArmAngle(ArmIdle);
                        if(Arm.getCurrentArmAngle() < 160)
                            Arm.setPivotAngle(PivotIdle);
                        Elevator.setTargetPosition(ElevatorUp + 100);
                        return Arm.motionCompleted();
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
                        return true;
                    }
                })
        ;
        transfer
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        if(!ActiveIntake.isOff()){
                            return false;
                        }
                        Elevator.PowerOnDownToTakeSample = true;
                        Extendo.PowerOnToTransfer = true;
                        ActiveIntake.powerOn();
                        return true;
                    }
                })
                .waitSeconds(0.2)
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
                        Extendo.PowerOnToTransfer = false;
                        Elevator.setTargetPosition(ElevatorUp);
                        if(Elevator.getCurrentPosition() > ElevatorUp - 80){
                            Arm.setArmAngle(ArmUpSample);
                            Arm.setPivotAngle(PivotUpSample);
                        }
                        return Arm.getCurrentArmAngle() >= 100;
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

        toScoreSample
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(ElevatorScoreSample);
                        return Elevator.ReachedTargetPosition();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(ArmScoreSample);
                        Arm.setPivotAngle(PivotScoreSample);
                        return Arm.motionCompleted();
                    }
                })
        ;
        ScoreSample
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        return true;
                    }
                })
                .waitSeconds(0.3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Retract.update();
                        return Retract.done();
                    }
                })
                ;
        ToTakeSpecimen
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(ElevatorUp);
                        if(Elevator.getCurrentPosition() > ElevatorUp - 80){
                            Arm.setArmAngle(ArmTakeSpecimen);
                        }
                        if(Arm.getCurrentArmAngle() > 90){
                            Arm.setPivotAngle(PivotTakeSpecimen);
                            Elevator.Disable = true;
                            Elevator.motor.setPower(-1);
                            Elevator.setTargetPosition(0);
                        }
                        return Elevator.getCurrentPosition() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.Disable = true;
                        Elevator.motor.setPower(-1);
                        return true;
                    }
                });
        TransferSpecimen
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = true;
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
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.PowerOnDownToTakeSample = false;
                        Elevator.setTargetPosition(ElevatorScoreSpecimen);
                        Arm.setArmAngle(ArmScoreSpecimen);
                        if(Arm.getCurrentArmAngle() < 250) Arm.setPivotAngle(PivotScoreSpecimen);
                        return Arm.motionCompleted() && Elevator.ReachedTargetPosition();
                    }
                })
                ;
        ScoreSpecimenReTake
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
                        ToTakeSpecimen.update();
                        return ToTakeSpecimen.done();
                    }
                })
        ;
    }

    public static void update(){
        if(currentTask.done()) {

            switch (CurrentState) {
                case IDLE:
                    if (Controls.GrabSpecimen) {
                        currentTask = ToTakeSpecimen;
                        Controls.GrabSpecimen = false;
                        CurrentState = States.IDLE_TAKE_SPECIMEN;
                    }
                    if (Storage.getStorageStatus() == Storage.SpecimenType.YELLOW && Controls.Transfer) {
                        currentTask = transfer;
                        CurrentState = States.IDLE_WITH_SAMPLE;
                        Controls.Transfer = false;
                    }
                case IDLE_WITH_SAMPLE:
                    if (Controls.ScoreLevel1) {
                        currentTask = toScoreSample;
                        ElevatorScoreSample = ElevatorScoreSample1;
                        Controls.ScoreLevel1 = false;
                        CurrentState = States.IDLE_SCORE_SAMPLE;
                    }
                    if (Controls.ScoreLevel2) {
                        currentTask = toScoreSample;
                        ElevatorScoreSample = ElevatorScoreSample1;
                        Controls.ScoreLevel2 = false;
                        CurrentState = States.IDLE_SCORE_SAMPLE;
                    }
                    break;
                case IDLE_SCORE_SAMPLE:
                    if (Controls.Grab) {
                        currentTask = ScoreSample;
                        Controls.Grab = false;
                        CurrentState = States.IDLE;
                    }
                    break;
                case IDLE_TAKE_SPECIMEN:
                    if (Controls.Grab) {
                        currentTask = TransferSpecimen;
                        Controls.Grab = false;
                        CurrentState = States.IDLE_SCORE_SPECIMEN;
                    }
                    break;
                case IDLE_SCORE_SPECIMEN:
                    if (Controls.GrabSpecimen) {
                        currentTask = ScoreSpecimenReTake;
                        Controls.GrabSpecimen = false;
                    }
                    if (Controls.Grab) {
                        currentTask = ScoreSample;
                        Controls.Grab = false;
                    }
                    break;
            }
        }
        if(Controls.Retract){
            currentTask.removeAllTasks();
            currentTask = Retract;
            CurrentState = States.IDLE;
            Controls.Retract = false;
        }

        currentTask.update();
    }

}