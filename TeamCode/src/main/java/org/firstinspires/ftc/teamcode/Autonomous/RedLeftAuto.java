package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.TunablePose2d;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

import java.sql.Time;

@Autonomous
@Config
public class RedLeftAuto extends LinearOpMode {

    public enum States{
        GOTO_SCORE_SPECIMEN,
        WAIT_FOR_SPECIMEN_TO_SCORE,
        GOTO_TAKE_SAMPLE1_POS1,
        GOTO_TAKE_SAMPLE1_POS2,
        EXTEND_EXTENDO,
        INTAKE_SAMPLE,
        RETRACT_EXTENDO,
        GOTO_SCORE_BASKET,
        WAIT_TO_SCOREBASKET,
        GO_TO_TAKE_SAMPLE1,
        GO_TO_TAKE_SAMPLE2
    }
    public enum Actions {
        NEXT,
        RETURN
    }
    public static States CurrentState = States.GOTO_SCORE_SPECIMEN;
    public static Actions Action;

    public static TunablePose2d ScoreSpecimen, Sample1Pos1, Sample1Pos2, Sample2, Sample3, BasketScore;
    public static double distanceToExtendExtendoForSample1, distanceToExtendExtendoForSample2, distanceToExtendExtendoForSample3;

    private static void ChangeStateTo(States state){
        CurrentState = state;
        TimeSinceStateStartedRunning.reset();
    }
    public static ElapsedTime TimeSinceStateStartedRunning = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeRobot(hardwareMap);
        OutTakeStateMachine.CurrentState = OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE;
        ActiveIntake.UnblockIntake();
        CurrentState = States.GOTO_SCORE_SPECIMEN;
        Action = Actions.NEXT;

        while (opModeInInit()){
            Initialization.updateCacheing();

            OutTakeController.Update();
            IntakeController.Update();
            Elevator.update();
            Arm.update();
            Extendo.update();
        }

        while (opModeIsActive()){
            Initialization.updateCacheing();


            switch (CurrentState){
                case GOTO_SCORE_SPECIMEN:
                    OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
                    Chassis.setTargetPosition(ScoreSpecimen);
                    if(!Chassis.ReachedTargetPosition(30)) break;
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    ChangeStateTo(States.WAIT_FOR_SPECIMEN_TO_SCORE);
                    break;
                case WAIT_FOR_SPECIMEN_TO_SCORE:
                    if(OutTakeStateMachine.CurrentState != OutTakeStateMachine.OutTakeStates.IDLE) {
                        TimeSinceStateStartedRunning.reset();
                        break;
                    }
                    if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                    ChangeStateTo(States.GOTO_TAKE_SAMPLE1_POS1);
                    break;
                case GOTO_TAKE_SAMPLE1_POS1:
                    Chassis.setTargetPosition(Sample1Pos1);
                    if(!Chassis.ReachedTargetPosition(100)) break;
                    ChangeStateTo(States.GOTO_TAKE_SAMPLE1_POS2);
                    break;
                case GO_TO_TAKE_SAMPLE2:
                    if(Action == Actions.NEXT) {
                        ChangeStateTo(States.EXTEND_EXTENDO);
                        break;
                    }
                    break;
                case EXTEND_EXTENDO:
                    if(Action == Actions.NEXT){
                        Extendo.Extend(Extendo.MaxExtendoExtension);
                        ChangeStateTo(States.INTAKE_SAMPLE);
                        break;
                    }
                    if(!Storage.hasAlliancePice()){
                        break;
                    }
                    if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE) {
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_IDLE_WITH_SAMPLE);
                        ChangeStateTo(States.GOTO_SCORE_BASKET);
                        Action = Actions.NEXT;
                    }
                    break;
                case INTAKE_SAMPLE:
                    ActiveIntake.powerOn();
                    Action = Actions.RETURN;
                    ChangeStateTo(States.EXTEND_EXTENDO);
                    break;
                case GOTO_SCORE_BASKET:
                    Chassis.setTargetPosition(BasketScore);
                    if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WITH_SAMPLE) {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        TimeSinceStateStartedRunning.reset();
                        break;
                    }
                    if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SAMPLE_SCORE && Chassis.ReachedTargetPosition(150)){
                        if(TimeSinceStateStartedRunning.seconds() >= 0.2) {
                            OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        }
                    } else TimeSinceStateStartedRunning.reset();
                    break;
            }


//            OutTakeController.Update();
            OutTakeStateMachine.Update(null);
            IntakeController.Update();
            Elevator.update();
            Arm.update();
            Extendo.update();
        }
    }

}
