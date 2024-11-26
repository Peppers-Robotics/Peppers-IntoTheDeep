package org.firstinspires.ftc.teamcode.OutTake;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;

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
            action = OutTakeStateMachine.OutTakeActions.RETRACT;
            Controls.Retract = false;
        }
        if(Controls.Grab || Controls.DunkToScore){
            action = OutTakeStateMachine.OutTakeActions.SCORE;
            Controls.Grab = false;
            Controls.DunkToScore = false;
        }
        if(Controls.IdleWithSample){
            OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_IDLE_WITH_SAMPLE);
            Controls.IdleWithSample = false;
        }
        OutTakeStateMachine.Update(action);
    }
}
