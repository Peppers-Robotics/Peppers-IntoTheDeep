package org.firstinspires.ftc.teamcode.OutTake;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;

public class OutTakeController extends GenericController {
    public static void Update(){
        OutTakeStateMachine.OutTakeActions action = null;
        if(Controls.ScoreLevel1){
            action = OutTakeStateMachine.OutTakeActions.SAMPLE;
            OutTakeStateMachine.ElevatorScoreSample = OutTakeStateMachine.ElevatorSample1;
            OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen1;
            Controls.ScoreLevel1 = false;
        } else if(Controls.ScoreLevel2){
            action = OutTakeStateMachine.OutTakeActions.SAMPLE;
            OutTakeStateMachine.ElevatorScoreSample = OutTakeStateMachine.ElevatorSample2;
            OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
            Controls.ScoreLevel2 = false;
        }
        if(Controls.GrabSpecimen){
            action = OutTakeStateMachine.OutTakeActions.SPECIMEN;
            Controls.GrabSpecimen = false;
        }
        if(Controls.Retract){
//            action = OutTakeStateMachine.OutTakeActions.RETRACT;
            OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.RETRACT_ARM);
            Controls.Retract = false;
        }
        if(Controls.Grab || Controls.DunkToScore){
            if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE) {

                Claw.close();
            }else if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE){
                OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.TRANSFER_ARM);
            } else {
                action = OutTakeStateMachine.OutTakeActions.SCORE;
            }
            Controls.Grab = false;
            Controls.DunkToScore = false;
        }
        if(Controls.IdleWithSample){
            OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.TRANSFER_ARM);
            Controls.IdleWithSample = false;
        }
        if(Controls.Throw){
            if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WITH_SAMPLE) {
                OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.THROW);
            }
            Controls.Throw = false;
        }
        OutTakeStateMachine.Update(action);
    }
}
