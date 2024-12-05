package org.firstinspires.ftc.teamcode.OutTake;

/*

IDLE:
    1 - ELEVATOR_TO_SAMPLE_SCORE
    2 - ELEVATOR_TAKE_SPECIMEN
ELEVATOR_SCORE_SAMPLE
    1 - ARM_SCORE_SAMPLE
ELEVATOR_

* */

import android.text.format.Time;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Actions;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.States;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;

@Config
public class OutTakeStateMachine {
    public static double IdleArmAngle = 21.3, IdlePivotAngle = 0, IdleElevatorLevel = -100, SafeElevatorLevel = 50;
    public static double IdleArmAngle_Sample = 200, IdlePivotAngle_Sample = 200;
    public static double ArmScoreSample = 245, PivotScoreSample = 200, ElevatorScoreSample;
    public static double ArmScoreSpecimen = 80, PivotScoreSpecimen = 0, ElevatorScoreSpecimen = 600, ArmPushSpecimen = 10, ElevatorPushSpecimen = 300;
    public static double ArmTakeSpecimen = 320, PivotTakeSpecimen = 198, ElevatorTakeSpecimen = 60; // DONE
    public static double ElevatorSpecimen1 = 100, ElevatorSpecimen2 = 600, ElevatorSample1 = 500, ElevatorSample2 = 1075;
    public static double ArmThrow = 300, ArmTrowRelease = 295;
    public static double TransferArm = 60, TransferPivot = 0;
    public enum OutTakeStates implements States {
        IDLE,
        TRANSFER_ARM,
        TRANSFER_EXTENDO,
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
        SCORE_SPECIMEN_ELEVATOR,
        THROW,
        TAKE_SPECIMEN
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
    public static boolean inAuto = false;
    public static void ChangeStateTo(OutTakeStates state){
        TimeSinceStateStartedRunning.reset();
        CurrentState = state;
    }
    public static boolean IDLEElevatorToPos1 = false;

    public static void Update(OutTakeActions action){
        if(action != null)
            CurrentAction = action;
        else CurrentAction = OutTakeActions.NEXT;
        switch (CurrentState){
            case IDLE:
                IDLEElevatorToPos1 = false;
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                Elevator.setTargetPosition(IdleElevatorLevel);
                if(!Arm.motionCompleted()) break;
                switch (CurrentAction){
                    case NEXT:
                    case NULL:
                    case RETRACT:
                        break;
                    case SAMPLE:
                    case SCORE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SAMPLE_SCORE);
                        break;
                    case SPECIMEN:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                        break;
                }
                break;
            case TRANSFER_ARM:

                Elevator.PowerOnDownToTakeSample = true;
                IntakeController.optimization = false;
                if(TimeSinceStateStartedRunning.seconds() < 0.05) break;
                Claw.close();
                if(TimeSinceStateStartedRunning.seconds() < 0.05 + 0.1) break;
                IntakeController.optimization = true;
                if(TimeSinceStateStartedRunning.seconds() < 0.05 + 0.15) break;
                Elevator.PowerOnDownToTakeSample = false;
                Elevator.setTargetPosition(SafeElevatorLevel);
                if(Elevator.getCurrentPosition() < SafeElevatorLevel - 10) break;

                ChangeStateTo(OutTakeStates.IDLE_WITH_SAMPLE);
                break;

            case IDLE_WITH_SAMPLE:
                Claw.close();
                Arm.setArmAngle(IdleArmAngle_Sample);
                Arm.setPivotAngle(IdlePivotAngle_Sample);
                if(Arm.getCurrentArmAngle() < 90){
                    break;
                }
                Elevator.setTargetPosition(IdleElevatorLevel);

                switch (CurrentAction){
                    case NULL:
                    case NEXT:
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                    case SAMPLE:
                    case SCORE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SAMPLE_SCORE);
                        break;
                    case SPECIMEN:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                        break;
                }
                break;
            case ELEVATOR_TO_SAMPLE_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSample);
                if(-Elevator.motor.getCurrentPosition() < SafeElevatorLevel) break;
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
                Arm.setPivotAngle(PivotScoreSample);
                switch (CurrentAction){
                    case NEXT:
                        if(Arm.getPrecentOfArmMotionCompleted() < 90) break;
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SAMPLE_SCORE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case ARM_TO_SPECIMEN_SCORE:
                Arm.setArmAngle(ArmScoreSpecimen);
                switch (CurrentAction){
                    case SCORE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_SCORE);
                        break;
                }
                break;
            case IDLE_WHILE_SPECIMEN_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                Arm.setArmAngle(ArmScoreSpecimen);
                Arm.setPivotAngle(PivotScoreSpecimen);
                if(!Elevator.ReachedTargetPosition()) break;
                if(!Arm.motionCompleted()) break;
                switch (CurrentAction){
                    case SCORE:
                        ChangeStateTo(OutTakeStates.SCORE_SPECIMEN_ELEVATOR);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case IDLE_WHILE_SAMPLE_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSample);
                if(!Elevator.ReachedTargetPosition()) break;
                Arm.setArmAngle(ArmScoreSample);
                Arm.setPivotAngle(PivotScoreSample);
                if(!Arm.motionCompleted()) break;
//                if(Elevator.getCurrentPosition() < ElevatorScoreSample - 20) break;
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
                if(!Arm.motionCompleted() && !inAuto) break;
                if(!Elevator.ReachedTargetPosition() && !inAuto) break;
//                if(Claw.HasElementInIt()){
//                    Claw.close();
//                    if(TimeSinceStateStartedRunning.seconds() >= 0.4) CurrentAction = OutTakeActions.NEXT;
//                }
                switch (CurrentAction){
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                    case SCORE:
                        ChangeStateTo(OutTakeStates.TAKE_SPECIMEN);
                        break;
                }
                break;
            case TAKE_SPECIMEN:
                Elevator.PowerOnDownToTakeSample = true;
                if(TimeSinceStateStartedRunning.seconds() < 0.1) break;
                Claw.close();

                if(TimeSinceStateStartedRunning.seconds() < 0.1 + 0.15) break;
                Elevator.PowerOnDownToTakeSample = false;

                ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE);
                break;
            case ARM_TO_SPECIMEN_TAKE:
                Elevator.setTargetPosition(0);
                if(Elevator.ReachedTargetPosition()) {
                    Arm.setArmAngle(ArmTakeSpecimen);
                    Arm.setPivotAngle(PivotTakeSpecimen);
                }
                if(Arm.getCurrentArmAngle() < 180) break;
                switch (CurrentAction) {
                    case NEXT:
                        Elevator.setTargetPosition(0);
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE);
                        break;
                }
                break;
            case RETRACT_ELEVATOR:
                Claw.open();
                Elevator.setTargetPosition(0);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.IDLE);
                        break;
                }
                break;
            case RETRACT_ARM:
                Claw.open();
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
                if(TimeSinceStateStartedRunning.seconds() < 0.15) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case SCORE_SPECIMEN_ARM:
                Arm.setArmAngle(ArmPushSpecimen);
                if(!Arm.motionCompleted()) break;
                Claw.open();
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case SCORE_SPECIMEN_ELEVATOR:
                Elevator.setTargetPosition(IdleElevatorLevel);
                if(Elevator.getCurrentPosition() > ElevatorScoreSpecimen - ElevatorPushSpecimen) break;
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.SCORE_SPECIMEN_ARM);
                        break;
                }
                break;
            case ELEVATOR_TO_SPECIMEN_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                if(!Elevator.ReachedTargetPosition()) break;
                switch (CurrentAction){
                    case NEXT:
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case THROW:
                Arm.setArmAngle(ArmThrow);
                if(Arm.getCurrentArmAngle() >= ArmTrowRelease){
                    Claw.open();
                    if(TimeSinceStateStartedRunning.seconds() >= 0.1) {
                        ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                    }
                } else TimeSinceStateStartedRunning.reset();
                break;
        }
        Initialization.telemetry.addData("Action", CurrentAction);
    }
}