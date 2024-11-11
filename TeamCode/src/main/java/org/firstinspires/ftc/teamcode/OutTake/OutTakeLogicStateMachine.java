package org.firstinspires.ftc.teamcode.OutTake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OutTakeLogicStateMachine {
    public static double IdleArmAngle, IdlePivotAngle, IdleElevatorLevel = 0;
    public static double ArmScoreSample, PivotScoreSample, ElevatorScoreSample;
    public static double ArmScoreSpecimen, PivotScoreSpecimen, ElevatorScoreSpecimen, ArmPushSpecimen;
    public static double ArmTakeSpecimen, PivotTakeSpecimen, ElevatorTakeSpecimen;
    public static double ElevatorSpecimen1, ElevatorSpecimen2, ElevatorSample1, ElevatorSample2;
    private static boolean DunkBoolean = false;
    public enum States{
        EXTEND_TO_SCORE_SPECIMEN,
        EXTEND_TO_SCORE_SAMPLE,
        EXTEND_TO_TAKE_SPECIMEN,
        PUSH_SPECIMEN,
        RETRACT,
        IDLE
    }
    public static States CurrentState;
    public static boolean canDunk = false;
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
            case IDLE:
                if(OutTakeController.ScoreSpecimen && Claw.HasElementInIt())
                    Claw.close();
                break;

            case RETRACT:
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                if(Arm.getPrecentOfArmMotionCompleted() >= 40) {
                    Elevator.setTargetPosition(IdleElevatorLevel);
                }
                OutTakeController.ScoreSpecimen = false;
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) CurrentState = States.IDLE;
                break;
            case EXTEND_TO_SCORE_SPECIMEN:
                Arm.setArmAngle(ArmScoreSpecimen);
                Arm.setPivotAngle(PivotScoreSpecimen);
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()){
                    CurrentState = States.IDLE;
                    canDunk = true;
                }
                break;
            case PUSH_SPECIMEN:
                canDunk = false;
                if(!DunkBoolean) {
                    Arm.setArmAngle(ArmPushSpecimen);
                    Arm.update();
                    if(Arm.motionCompleted()){ DunkBoolean = true; time.reset();}
                } else {
                    if(time.seconds() >= 0.2){
                        Claw.open();
                        Arm.setPivotAngle(ArmScoreSpecimen);
                        if(Arm.motionCompleted()) {
                            CurrentState = States.RETRACT;
                            DunkBoolean = false;
                        }
                    }
                }
                break;
            case EXTEND_TO_TAKE_SPECIMEN:
                Arm.setArmAngle(ArmTakeSpecimen);
                Arm.setPivotAngle(PivotTakeSpecimen);
                Elevator.setTargetPosition(ElevatorTakeSpecimen);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()) CurrentState = States.IDLE;
                break;
            case EXTEND_TO_SCORE_SAMPLE:
                Arm.setArmAngle(ArmScoreSample);
                Arm.setPivotAngle(PivotScoreSample);
                Elevator.setTargetPosition(ElevatorScoreSample);
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) CurrentState = States.IDLE;
                break;
        }
    }

}