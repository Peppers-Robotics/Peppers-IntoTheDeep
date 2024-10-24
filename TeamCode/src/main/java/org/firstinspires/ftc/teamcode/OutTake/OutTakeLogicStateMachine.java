package org.firstinspires.ftc.teamcode.OutTake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OutTakeLogicStateMachine {

    // IDLE STATE
    public static double IdleArmRotation = 0, IdlePivotRotation = 0, IdleElevatorPos = 0;

    // GRAB SAMPLE STATE
    public static double GrabSpecimenArmRotation = 0, GrabSpecimenPivotRotation = 0, GrabSpecimenElevatorPos = 0;
    public enum States{
        IDLE,
        GRAB_SAMPLE,
        GRAB_SPECIMEN,
        SCORE_SPECIMEN,
        SCORE_SAMPLE
    }
    private static States OutTakeState;
    private static int basketToScore, barToScore;
    private static ElapsedTime time;
    static {
        OutTakeState = States.IDLE;
        basketToScore = 1;
        barToScore = 1;
        time = new ElapsedTime();
    }

    public static void ScoreSample(int basket) throws Exception {
        if(basket < 1 || basket > 2) throw new Exception();
        basketToScore = basket;
        time.reset();

    }
    public static void ScoreSpecimen(int bar) throws Exception {
        if(bar < 1 || bar > 2) throw new Exception();
        barToScore = bar;
        time.reset();
    }


    synchronized public static void update(){
        switch (OutTakeState){
            case IDLE:
                Arm.setPivotAngle(IdlePivotRotation);
                Arm.setArmAngle(IdleArmRotation);
                Elevator.setTargetPosition(IdleElevatorPos);
                Claw.open();
                break;
            case GRAB_SAMPLE:
                if(Elevator.getTargetPosition() != GrabSpecimenElevatorPos) time.reset();
                Elevator.setTargetPosition(GrabSpecimenElevatorPos);
                break;
            case GRAB_SPECIMEN:
                break;
            case SCORE_SPECIMEN:
                break;
            case SCORE_SAMPLE:
                break;
        }
    }
}
