package org.firstinspires.ftc.teamcode.OutTake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OutTakeLogicStateMachine {
    public static double IdleArmAngle, IdlePivotAngle, IdleElevatorLevel = 0;
    public static double ArmScoreSample, PivotScoreSample, ElevatorScoreSample;
    public static double ArmScoreSpecimen, PivotScoreSpecimen, ElevatorScoreSpecimen, ArmPushSpecimen;
    public static double ArmTakeSpecimen, PivotTakeSpecimen, ElevatorTakeSpecimen;
    public static double ElevatorSpecimen1, ElevatorSpecimen2, ElevatorSample1, ElevatorSample2;
    public enum States{
        EXTEND_TO_SCORE_SPECIMEN,
        EXTEND_TO_SCORE_SAMPLE,
        EXTEND_TO_TAKE_SPECIMEN,
        PUSH_SPECIMEN,
        RETRACT,
        IDLE
    }
    public static States CurrentState;
    private static ElapsedTime time;
    static {
        CurrentState = States.IDLE;
        time = new ElapsedTime();
    }
    public static void ChangeState(States state){
        if(CurrentState != States.IDLE) return;
        CurrentState = state;
    }

    public static void Update(){
        switch (CurrentState){
            case IDLE: break;

            case RETRACT:
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                Elevator.setTargetPosition(IdleElevatorLevel);
                OutTakeController.ScoreSpecimen = false;
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) CurrentState = States.IDLE;
                break;
            case EXTEND_TO_SCORE_SPECIMEN:
                Arm.setArmAngle(ArmScoreSpecimen);
                Arm.setPivotAngle(PivotScoreSpecimen);
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()){
                    CurrentState = States.PUSH_SPECIMEN;
                    time.reset();
                }
                break;
            case PUSH_SPECIMEN:
                if(time.seconds() >= 0.1 && Claw.isClosed()) {
                    Arm.setArmAngle(ArmPushSpecimen);
                    time.reset();
                }
                if(Arm.getCurrentArmAngle() == ArmPushSpecimen && time.seconds() >= 0.1){
                    Arm.setArmAngle(ArmScoreSample);
                    Claw.open();
                    time.reset();
                }
                if(Arm.getCurrentArmAngle() == ArmScoreSample && time.seconds() >= 0.1){
                    CurrentState = States.RETRACT;
                }
                break;
            case EXTEND_TO_TAKE_SPECIMEN:
                Arm.setArmAngle(ArmTakeSpecimen);
                Arm.setPivotAngle(PivotTakeSpecimen);
                Elevator.setTargetPosition(ElevatorTakeSpecimen);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()) CurrentState = States.IDLE;
                break;
        }
    }

}