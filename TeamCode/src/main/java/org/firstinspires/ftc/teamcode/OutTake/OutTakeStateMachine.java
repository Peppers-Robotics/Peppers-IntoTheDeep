package org.firstinspires.ftc.teamcode.OutTake;

/*

IDLE:
    1 - ELEVATOR_TO_SAMPLE_SCORE
    2 - ELEVATOR_TAKE_SPECIMEN
ELEVATOR_SCORE_SAMPLE
    1 - ARM_SCORE_SAMPLE
ELEVATOR_

* */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Actions;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.OutTakeFSM;
import org.firstinspires.ftc.teamcode.HelperClasses.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.States;

@Config
public class OutTakeStateMachine {
    public static double IdleArmAngle = 0, IdlePivotAngle = 0, IdleElevatorLevel = -100, SafeElevatorLevel = 40;
    public static double IdleArmAngle_Specimen = 180, IdlePivotAngle_Specimen = 0;
    public static double ArmScoreSample = 225, PivotScoreSample = 200, ElevatorScoreSample;
    public static double ArmScoreSpecimen = 160, PivotScoreSpecimen = 0, ElevatorScoreSpecimen = 400, ArmPushSpecimen = 0;
    public static double ArmTakeSpecimen = 270, PivotTakeSpecimen = 200, ElevatorTakeSpecimen = 10; // DONE
    public static double ElevatorSpecimen1 = 100, ElevatorSpecimen2 = 260, ElevatorSample1 = 500, ElevatorSample2 = 1200;
    public enum OutTakeStates implements States {
        IDLE,
        IDLE_WITH_SAMPLE,
        ELEVATOR_TO_SAMPLE_SCORE,
        ELEVATOR_TO_SPECIMEN_SCORE,
        ELEVATOR_TO_SPECIMEN_TAKE,
        ARM_TO_SAMPLE_SCORE,
        ARM_TO_SPECIMEN_SCORE,
        IDLE_WHILE_SPECIMEN_SCORE,
        IDLE_WHILE_SAMPLE_SCORE,
        IDLE_WHILE_SPECIMEN_TAKE,
        ARM_TO_SPECIMEN_TAKE,
        RETRACT_ELEVATOR,
        RETRACT_ARM,
        SCORE_SAMPLE,
        SCORE_SPECIMEN_ARM,
        SCORE_SPECIMEN_ELEVATOR
    }
    public enum OutTakeActions implements Actions {
        NULL,
        SAMPLE,
        SPECIMEN,
        RETRACT,
        NEXT,
        SCORE
    }

    public static OutTakeStates CurrentState = OutTakeStates.IDLE;
    public static OutTakeActions CurrentAction = OutTakeActions.NULL;
    public static ElapsedTime TimeSinceStateStartedRunning = new ElapsedTime();
    public static void ChangeStateTo(OutTakeStates state){
        TimeSinceStateStartedRunning.reset();
        CurrentState = state;
    }


    public static void Update(OutTakeActions action){
        if(action != null)
            CurrentAction = action;
        else CurrentAction = OutTakeActions.NEXT;
        switch (CurrentState){
            case IDLE:
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                if(!Arm.motionCompleted()) break;
                switch (CurrentAction){
                    case NEXT:
                    case NULL:
                    case RETRACT:
                        break;
                    case SAMPLE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SAMPLE_SCORE);
                        break;
                    case SPECIMEN:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                        break;
                }
                break;
            case IDLE_WITH_SAMPLE:
                Arm.setPivotAngle(IdlePivotAngle_Specimen);
                Arm.setArmAngle(IdleArmAngle_Specimen);
                if(!Arm.motionCompleted()) break;
                switch (CurrentAction){
                    case NULL:
                    case NEXT:
                    case RETRACT:
                        break;
                    case SAMPLE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SAMPLE_SCORE);
                        break;
                    case SPECIMEN:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                        break;
                }
                break;
            case ELEVATOR_TO_SAMPLE_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSample);
                if(Elevator.getCurrentPosition() < SafeElevatorLevel) break;
                switch (CurrentAction){
                    case NULL:
                    case SPECIMEN:
                    case SAMPLE:
                        break;
                    case NEXT:
                        ChangeStateTo(OutTakeStates.ARM_TO_SAMPLE_SCORE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                        break;
                }
                break;
            case ELEVATOR_TO_SPECIMEN_TAKE:
                Elevator.setTargetPosition(ElevatorTakeSpecimen);
                if(Elevator.getCurrentPosition() < SafeElevatorLevel) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.ARM_TO_SPECIMEN_TAKE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                        break;
                }
                break;
            case ARM_TO_SAMPLE_SCORE:
                Arm.setArmAngle(ArmScoreSample);
                Arm.setPivotAngle(PivotScoreSample);
                switch (CurrentAction){
                    case NEXT:
                        if(!Arm.motionCompleted()) break;
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SAMPLE_SCORE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case ARM_TO_SPECIMEN_SCORE:
                Arm.setArmAngle(ArmPushSpecimen);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_SCORE);
                        break;
                }
                break;
            case IDLE_WHILE_SPECIMEN_SCORE:
                if(!Arm.motionCompleted()) break;
                if(!Elevator.ReachedTargetPosition()) break;
                switch (CurrentAction){
                    case SCORE:
                        ChangeStateTo(OutTakeStates.SCORE_SPECIMEN_ARM);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case IDLE_WHILE_SAMPLE_SCORE:
                if(!Arm.motionCompleted()) break;
                if(!Elevator.ReachedTargetPosition()) break;
                switch (CurrentAction){
                    case SCORE:
                        ChangeStateTo(OutTakeStates.SCORE_SAMPLE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case IDLE_WHILE_SPECIMEN_TAKE:
                if(!Arm.motionCompleted()) break;
                if(!Elevator.ReachedTargetPosition()) break;
                CurrentAction = OutTakeActions.NULL;
                if(Claw.HasElementInIt()){
                    Claw.close();
                    if(TimeSinceStateStartedRunning.seconds() >= 0.1) CurrentAction = OutTakeActions.NEXT;
                }
                switch (CurrentAction){
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                    case NEXT:
                        ChangeStateTo(OutTakeStates.ARM_TO_SPECIMEN_SCORE);
                        break;
                }
                break;
            case ARM_TO_SPECIMEN_TAKE:
                Elevator.setTargetPosition(SafeElevatorLevel);
                if(Elevator.ReachedTargetPosition()) {
                    Arm.setArmAngle(ArmTakeSpecimen);
                    Arm.setPivotAngle(PivotTakeSpecimen);
                }
                if(Arm.getCurrentArmAngle() < 30) break;
                switch (CurrentAction) {
                    case NEXT:
                        Elevator.setTargetPosition(0);
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE);
                        break;
                }
                break;
            case RETRACT_ELEVATOR:
                Elevator.setTargetPosition(0);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.IDLE);
                        break;
                }
                break;
            case RETRACT_ARM:
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                if(Arm.getCurrentArmAngle() >= 30) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                        break;
                }
                break;
            case SCORE_SAMPLE:
                Claw.open();
                if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case SCORE_SPECIMEN_ARM:
                Arm.setArmAngle(ArmPushSpecimen);
                if(Arm.getPrecentOfArmMotionCompleted() < 70) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.SCORE_SPECIMEN_ELEVATOR);
                        break;
                }
                break;
            case SCORE_SPECIMEN_ELEVATOR:
                Elevator.setTargetPosition(0);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case ELEVATOR_TO_SPECIMEN_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE);
                        break;
                }
                break;
        }
    }


}
