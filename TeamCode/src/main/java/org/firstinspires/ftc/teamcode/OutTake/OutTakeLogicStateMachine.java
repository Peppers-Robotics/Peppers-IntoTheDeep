package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class OutTakeLogicStateMachine {
    public static double IdleArmAngle = 0, IdlePivotAngle = 0, IdleElevatorLevel = -100;
    public static double ArmScoreSample = 225, PivotScoreSample = 200, ElevatorScoreSample;
    public static double ArmScoreSpecimen = 160, PivotScoreSpecimen = 0, ElevatorScoreSpecimen = 400, ArmPushSpecimen = 0;
    public static double ArmTakeSpecimen = 270, PivotTakeSpecimen = 200, ElevatorTakeSpecimen = 10; // DONE
    public static double ElevatorSpecimen1 = 100, ElevatorSpecimen2 = 260, ElevatorSample1 = 500, ElevatorSample2 = 1200;
    public static boolean DunkBoolean = false;
    public enum States{
        EXTEND_TO_SCORE_SPECIMEN,
        EXTEND_TO_SCORE_SAMPLE,
        EXTEND_TO_TAKE_SPECIMEN,
        PUSH_SPECIMEN,
        RETRACT,
        IDLE,
        IDLE_SCORING
    }
    public static States CurrentState;
    public static boolean canDunk = false;
    private static ElapsedTime time;

    static {
        CurrentState = States.IDLE;
        time = new ElapsedTime();
    }
    public static void ResetForTeleOp(){
        sense = false;
        isScoringSample = false;
        Claw.open();
        Arm.setArmAngle(IdleArmAngle);
        Arm.setPivotAngle(IdlePivotAngle);
        canDunk = false;
    }
    synchronized public static void ChangeState(States state){
        if(CurrentState != States.IDLE) return;
        CurrentState = state;
    }
    public static void ChangeState(States state, double s){
        new Thread(() -> {
            long t = System.currentTimeMillis();
            while(System.currentTimeMillis() - t < s / 1000.f);
            ChangeState(state);
        }).start();
    }
    private static boolean sense = false;
    public static boolean isScoringSample = false;

    public static boolean isWaitingForSample(){
        return CurrentState == States.IDLE && Arm.getCurrentArmAngle() == IdleArmAngle && Arm.getCurrentPivotAngle() == IdlePivotAngle;
    }

    public static void Update(){
        switch (CurrentState){
            case IDLE:
                if(sense && Claw.HasElementInIt()) {
                    Claw.close();
                    sense = false;
                }
                break;
            case IDLE_SCORING:
                Claw.close();
                Arm.setArmAngle(180);
                Arm.setPivotAngle(0);
                CurrentState = States.IDLE;
                break;
            case RETRACT:
                Claw.open();
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                if(Arm.armProfile.motionEnded()) {
                    Elevator.setTargetPosition(IdleElevatorLevel);
                }
                OutTakeController.ScoreSpecimen = false;
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) CurrentState = States.IDLE;
                break;
            case EXTEND_TO_SCORE_SPECIMEN:
                sense = false;
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                if(Elevator.getCurrentPosition() >= 50){
                    Arm.setArmAngle(ArmScoreSpecimen);
                    Arm.setPivotAngle(PivotScoreSpecimen);
                }
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()){
                    CurrentState = States.IDLE;
                    canDunk = true;
                }
                break;
            case PUSH_SPECIMEN:
                canDunk = false;
                sense = false;
                if(Arm.getCurrentArmAngle() != ArmPushSpecimen) {
                    Arm.setArmAngle(ArmPushSpecimen);
                    Elevator.setTargetPosition(ElevatorScoreSpecimen - 30);
                    Arm.update();
                    time.reset();
                } else {
                    if(time.seconds() >= 0.4 && Claw.isClosed()) {
                        Claw.open();
                        time.reset();
                    }
                    else if(time.seconds() >= 0.2 && !Claw.isClosed()){
                        CurrentState = States.RETRACT;
                    }
                }
                break;
            case EXTEND_TO_TAKE_SPECIMEN:
                Arm.setArmAngle(ArmTakeSpecimen);
                Arm.setPivotAngle(PivotTakeSpecimen);
                Elevator.setTargetPosition(ElevatorTakeSpecimen);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()) CurrentState = States.IDLE;
                sense = true;
                break;
            case EXTEND_TO_SCORE_SAMPLE:
                sense = false;
                isScoringSample = true;
                Arm.setArmAngle(ArmScoreSample);
                Arm.setPivotAngle(PivotScoreSample);
                Elevator.setTargetPosition(ElevatorScoreSample);
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) CurrentState = States.IDLE;
                break;
        }
    }

}