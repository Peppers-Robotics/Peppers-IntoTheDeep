package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Actions;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.States;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;

@Config
public class OutTakeStateMachine {
    public static double IdleArmAngle = -35, IdlePivotAngle = 190, IdleElevatorLevel = -100, SafeElevatorLevel = 100;
    public static double IdleArmAngle_Sample = 180, IdlePivotAngle_Sample = 170;
    public static double ArmScoreSample = 210, PivotScoreSample = -20, ElevatorScoreSample;
    public static double ArmScoreSpecimen = 125, PivotScoreSpecimen = 180, ElevatorScoreSpecimen = 200, ArmPushSpecimen = 10, ElevatorPushSpecimen = 300;
    public static double ArmTakeSpecimen = 330, PivotTakeSpecimen = -30, ElevatorTakeSpecimen = 120; // DONE
    public static double ElevatorSpecimen1 = 490, ElevatorSpecimen2 = 200, ElevatorSample1 = 450, ElevatorSample2 = 980;
    public static double ArmThrow = 300, ArmTrowRelease = 300, d2power = 5, d2powerI = 3;
    public static double TransferArm = 60, TransferPivot = 0;
    public static boolean reatched = false;
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
        TAKE_SPECIMEN,
        AUTO_PARK
    }
    public enum OutTakeActions implements Actions {
        NULL,
        SAMPLE,
        SPECIMEN,
        RETRACT,
        NEXT,
        SCORE
    }

    public static OutTakeStates CurrentState = OutTakeStates.IDLE, defaultState = OutTakeStates.IDLE;
    public static OutTakeActions CurrentAction = OutTakeActions.NULL;
    public static ElapsedTime TimeSinceStateStartedRunning = new ElapsedTime();
    public static boolean inAuto = false, retractFromSample = false;
    public static void ChangeStateTo(OutTakeStates state){
        TimeSinceStateStartedRunning.reset();
        CurrentState = state;
    }
    public static boolean IDLEElevatorToPos1 = false, autoTakingSamples = false;

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
                ActiveIntake.powerOn();
                IntakeController.gamepad1.right_stick_y = 0;
                Elevator.PowerOnDownToTakeSample = true;
                IntakeController.optimization = false;
                if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                Claw.close();
                if(TimeSinceStateStartedRunning.seconds() < 0.2 + 0.12) break;
                IntakeController.optimization = true;
                Elevator.PowerOnDownToTakeSample = false;
                Elevator.setTargetPosition(SafeElevatorLevel);
                ActiveIntake.powerOff();
                //if(Elevator.getCurrentPosition() < SafeElevatorLevel - 10) break;

                ChangeStateTo(OutTakeStates.IDLE_WITH_SAMPLE);
                break;

            case IDLE_WITH_SAMPLE:
                Claw.close();
                Arm.setArmAngle(IdleArmAngle_Sample);
                Arm.setPivotAngle(IdlePivotAngle_Sample);
                if(Arm.getCurrentArmAngle() < 90){
                    break;
                }
                Elevator.setTargetPosition(SafeElevatorLevel);

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
                Claw.open();
                Elevator.setTargetPosition(SafeElevatorLevel + 100);
                if(Elevator.getCurrentPosition() < SafeElevatorLevel - 40 && Math.abs(Elevator.motor.getVelocity()) > 4) break;
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
                Arm.setPivotAngle(IdlePivotAngle_Sample);
                Arm.setArmAngle(IdleArmAngle_Sample);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SAMPLE_SCORE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case ARM_TO_SPECIMEN_SCORE:
                Arm.setArmAngle(ArmScoreSpecimen);
                if(Arm.getCurrentArmAngle() > ArmScoreSpecimen - 30) break;
                switch (CurrentAction){
                    case SCORE:
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_SCORE);
                        break;
                }
                break;
            case IDLE_WHILE_SPECIMEN_SCORE:
                if(!Controls.ImogenDriver) {
                    ElevatorScoreSpecimen -= Controls.gamepad2.right_stick_y * d2power;
                } else {
                    ElevatorScoreSpecimen += (Controls.gamepad1.gamepad.dpad_left ? 1 : 0) * (-d2powerI) + (Controls.gamepad1.gamepad.dpad_left ? 1 : 0) * (d2powerI);
                }
//                Elevator.setTargetPosition(Elevator.getTargetPosition() - Controls.gamepad2.right_stick_y * d2power);
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                Arm.setArmAngle(ArmScoreSpecimen);
                Arm.setPivotAngle(PivotScoreSpecimen);
                if(!Elevator.ReachedTargetPosition() && !reatched){
                    break;
                }
                if(!Arm.motionCompleted()) break;
//                if(OutTakeController.wasL2Activated) ElevatorSpecimen2 = ElevatorScoreSpecimen;
//                else ElevatorSpecimen1 = ElevatorScoreSpecimen;
                switch (CurrentAction){
                    case SPECIMEN:
                        Claw.open();
                        ChangeStateTo(OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                        break;
                    case SCORE:
//                        ChangeStateTo(OutTakeStates.SCORE_SPECIMEN_ELEVATOR);
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case IDLE_WHILE_SAMPLE_SCORE:
                Elevator.setTargetPosition(ElevatorScoreSample);

                if(Elevator.getCurrentPosition() < ElevatorScoreSample - 200 || Math.abs(Elevator.motor.getVelocity()) > 20) break;

                Arm.setArmAngle(ArmScoreSample);
                Arm.setPivotAngle(PivotScoreSample);

                if(!Arm.motionCompleted()) break;
                switch (CurrentAction){
                    case SCORE:
                        retractFromSample = true;
                        if(!inAuto) {
                            Initialization.localizer.reset();
                        }
                        ChangeStateTo(OutTakeStates.SCORE_SAMPLE);
                        break;
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case IDLE_WHILE_SPECIMEN_TAKE:
                Elevator.PowerOnDownToTakeSample = true;
                Elevator.power = 0.8;
                if(!Arm.motionCompleted() && !inAuto) break;
                Elevator.setTargetPosition(Elevator.getTargetPosition() - d2power * Controls.gamepad2.right_stick_y);
//                if(Elevator.getCurrentPosition() - Elevator.getTargetPosition() > 10 && !inAuto) break;
                if(Claw.HasElementInIt() && !inAuto){
                    Claw.close();
                    if(TimeSinceStateStartedRunning.seconds() >= 0.05) CurrentAction = OutTakeActions.SCORE;
                } else {
                    TimeSinceStateStartedRunning.reset();
                    Claw.open();
                }
                switch (CurrentAction){
                    case RETRACT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                    case SCORE:
                        ChangeStateTo(OutTakeStates.TAKE_SPECIMEN);
                        Elevator.power = 1;
                        break;
                }
                break;
            case TAKE_SPECIMEN:
                Elevator.PowerOnDownToTakeSample = true;
                Claw.close();

                if(TimeSinceStateStartedRunning.seconds() < 0.15) break;
                Elevator.PowerOnDownToTakeSample = false;

                ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE);
                Elevator.setTargetPosition(ElevatorScoreSpecimen);
                break;
            case ARM_TO_SPECIMEN_TAKE:
                Arm.setPivotAngle(PivotTakeSpecimen);
                Arm.setArmAngle(ArmTakeSpecimen - 45);
//                Elevator.PowerOnDownToTakeSample = true;
                if (Arm.getCurrentPivotAngle() > 20) break;
                Arm.setArmAngle(ArmTakeSpecimen);
                Elevator.setTargetPosition(IdleElevatorLevel);
                switch (CurrentAction) {
                    case NEXT:
                        ChangeStateTo(OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE);
                        break;
                }
                break;
            case RETRACT_ELEVATOR:
                Arm.setArmAngle(IdleArmAngle);
                Arm.setPivotAngle(IdlePivotAngle);
                Claw.open();
                Elevator.setTargetPosition(IdleElevatorLevel);
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
                if(!Arm.motionCompleted()) break;
                if(!inAuto) {
                    if (retractFromSample && (
                            !(Initialization.localizer.getPoseEstimate().getX() < -4 ||
                                    Math.abs(Initialization.localizer.getPoseEstimate().getY()) > 4) || action == OutTakeActions.RETRACT)) {
                        break;
                    }
                }
                retractFromSample = false;
                Elevator.setTargetPosition(IdleElevatorLevel);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                        break;
                }
                break;
            case SCORE_SAMPLE:
                Arm.setArmAngle(ArmScoreSample + 15);
                Claw.open();
                if(!Arm.motionCompleted()){
                    TimeSinceStateStartedRunning.reset();
                    break;
                }
                if(TimeSinceStateStartedRunning.seconds() < 0.1) return;
                Arm.setArmAngle(ArmScoreSample);
                switch (CurrentAction){
                    case NEXT:
                        ChangeStateTo(OutTakeStates.RETRACT_ARM);
                        break;
                }
                break;
            case SCORE_SPECIMEN_ARM:
//                Arm.setArmAngle(ArmPushSpecimen);
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
                if(Elevator.getCurrentPosition() < ElevatorScoreSpecimen - 20) break;
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
                    if(TimeSinceStateStartedRunning.seconds() >= 0.04) {
                        Claw.open();
                        if(TimeSinceStateStartedRunning.seconds() >= 0.04 + 0.08) {
                            ChangeStateTo(OutTakeStates.RETRACT_ELEVATOR);
                        }
                    }
                } else TimeSinceStateStartedRunning.reset();
                break;
            case AUTO_PARK:
                Elevator.setTargetPosition(ElevatorScoreSpecimen - 30);
                Arm.setArmAngle(ArmPark);
                break;
        }
    }
    public static double ArmPark = 125;
}