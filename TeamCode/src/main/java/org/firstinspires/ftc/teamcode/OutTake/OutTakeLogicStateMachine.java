package org.firstinspires.ftc.teamcode.OutTake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OutTakeLogicStateMachine {

    // IDLE STATE
    public static double IdleArmRotation = 0, IdlePivotRotation = 0, IdleElevatorPos = 0;

    // GRAB SAMPLE STATE
    public static double GrabSpecimenArmRotation = 0, GrabSpecimenPivotRotation = 0;
    public static double ScoreSpecimenArmRotation = 0, ScoreSpecimenPivotRotation = 0;
    public static double ScoreSampleArmRotation = 0, ScoreSamplePivotRotation = 0;

    public static double ElevatorBasket1 = 0, ElevatorBasket2 = 0, ElevatorChamber1 = 0, ElevatorChamber2 = 0;


    public enum States{
        IDLE,
        GRAB_SAMPLE,
        GRAB_SPECIMEN,
        SCORE_SPECIMEN,
        SCORE_SAMPLE,
        Noting,
        DUNK_TO_SCORE
    }
    private static States OutTakeState;
    private static int basketToScore, barToScore;
    private static ElapsedTime time;
    static {
        OutTakeState = States.Noting;
        basketToScore = 1;
        barToScore = 1;
        time = new ElapsedTime();
    }

    public static void SetState(States toSet){
        if(OutTakeState != States.Noting) return;
        OutTakeState = toSet;
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
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) OutTakeState = States.Noting;
                break;
            case GRAB_SAMPLE:

                Claw.close();
                OutTakeState = States.Noting;

                break;
            case GRAB_SPECIMEN:
                if(Arm.getCurrentArmAngle() == IdleArmRotation) time.reset();
                Arm.setArmAngle(GrabSpecimenArmRotation);
                Arm.setPivotAngle(GrabSpecimenPivotRotation);

                if(Arm.motionCompleted()){
                    OutTakeState = States.Noting;
                }

                break;
            case SCORE_SAMPLE:
                Arm.setPivotAngle(ScoreSpecimenPivotRotation);
                Arm.setArmAngle(ScoreSpecimenArmRotation);
                Elevator.setTargetPosition(basketToScore == 1 ? ElevatorBasket1 : ElevatorBasket2);
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) OutTakeState = States.Noting;

                break;
            case SCORE_SPECIMEN:
                Arm.setPivotAngle(ScoreSpecimenPivotRotation);
                Arm.setArmAngle(ScoreSpecimenArmRotation);
                Elevator.setTargetPosition(barToScore == 1 ? ElevatorChamber1 : ElevatorChamber2);
                if(Arm.motionCompleted() && Elevator.ReachedTargetPosition()) OutTakeState = States.Noting;


                break;
            case DUNK_TO_SCORE:
                Elevator.PIDControllerInWork = false;
                if(Elevator.motor.getPower() != -1) time.reset();
                Elevator.motor.setPower(-1);
                if(time.seconds() >= 0.05) {
                    Elevator.motor.setPower(0);
                    Elevator.PIDControllerInWork = true;
                }

                break;
            case Noting:

                break;
        }
    }
}
