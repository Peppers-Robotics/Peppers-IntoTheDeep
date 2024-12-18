package org.firstinspires.ftc.teamcode.OutTake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OutTakeStateMachineMK2 {
    public static double
    IdleArm = 0, IdleTurret = 0,
    ArmMiddle = 180, TurretMiddle = 180,
    ArmScoreSample = 230, TurretScoreSamplePlane = -45,
    ArmScoreSpecimen = 190, TurretScoreSpecimenPlane = 0,
    ArmTakeSpecimen = 320, TurretTakeSpecimenPlane = 90,
    ElevatorIdle = 0, ElevatorScoreSample = 200, ElevatorScoreSpecimen = 200, ElevatorTakeSpecimen = 0,

    ElevatorSample1 = 200, ElevatorSample2 = 600, ElevatorSpecimen1 = 200, ElevatorSpecimen2 = 330
    ;

    public enum States{
        IDLE,
        IDLE_WITH_SAMPLE,
        IDLE_WHILE_SPECIMEN_TAKE,
        IDLE_SCORE_SPECIMEN,
        IDLE_WHILE_SCORE,
        ARM_UP,
        ARM_TO_MIDDLE,
        ARM_TO_WALL,
        ELEVATOR_TO_POS,
        ARM_TO_SCORE,
        SCORE,
        RETRACT_ARM,
        RETRACT_ELEVATOR,
        TRANSFER,
        ARM_SWING_BACK,
        ARM_SWING_FORWARD,
    }
    public enum Actions{
        SAMPLE,
        SPECIMEN,
        SCORE,
        RETRACT,
    }
    public static States CurrentState = States.IDLE;
    public static Actions CurrentAction = Actions.RETRACT;
    public static ElapsedTime TimeSinceStateStartedRunning = new ElapsedTime();

    public static void ChangeState(States state){
        CurrentState = state;
        TimeSinceStateStartedRunning.reset();
    }
    public static void ChangeAction(Actions action){
        CurrentAction = action;
    }

    public static void Update(){
        switch (CurrentState){
            case IDLE:
                ArmMK2.stopHolding();
                ArmMK2.setArmAngle(IdleArm);
                ArmMK2.setTurretAngle(IdleTurret);
                Elevator.setTargetPosition(ElevatorIdle);
                if(CurrentAction == Actions.SPECIMEN){
                    ChangeState(States.ARM_TO_MIDDLE);
                }
                break;
            case ARM_TO_MIDDLE:
                if(CurrentAction == Actions.RETRACT){
                    ChangeState(States.RETRACT_ARM);
                    break;
                }
                ArmMK2.setArmAngle(ArmMiddle);
                ArmMK2.update();
                if(ArmMK2.ArmMotionCompleted()) ArmMK2.setTurretAngle(TurretMiddle);
                ArmMK2.update();

                if(CurrentAction == Actions.SPECIMEN && Arm.motionCompleted()){
                    ChangeState(States.ARM_TO_WALL);
                    break;
                }
                if(CurrentAction == Actions.SAMPLE){
                    ChangeState(States.ELEVATOR_TO_POS);
                }
                break;
            case ARM_TO_WALL:
                if(CurrentAction == Actions.RETRACT){
                    ChangeState(States.RETRACT_ARM);
                    break;
                }
                ArmMK2.setArmAngle(ArmTakeSpecimen);
                ArmMK2.update();
                if(ArmMK2.ArmMotionCompleted()) ArmMK2.setTurretHoldPlane(TurretTakeSpecimenPlane);
                ArmMK2.update();

                ChangeState(States.IDLE_WHILE_SPECIMEN_TAKE);
                break;
            case IDLE_WHILE_SPECIMEN_TAKE:
                if(CurrentAction == Actions.RETRACT){
                    ChangeState(States.RETRACT_ARM);
                    break;
                }
                if(CurrentAction == Actions.SCORE){

                }

                break;
        }
    }
}
