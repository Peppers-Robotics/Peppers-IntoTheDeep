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
        } else if(Controls.ScoreLevel2){
            action = OutTakeStateMachine.OutTakeActions.SAMPLE;
            OutTakeStateMachine.ElevatorScoreSample = OutTakeStateMachine.ElevatorSample2;
            OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
        }
        if(Controls.GrabSpecimen){
            action = OutTakeStateMachine.OutTakeActions.SPECIMEN;
        }
        if(Controls.Retract){
            action = OutTakeStateMachine.OutTakeActions.RETRACT;
        }
        if(Controls.Grab || Controls.DunkToScore){
            action = OutTakeStateMachine.OutTakeActions.SCORE;
        }
        OutTakeStateMachine.Update(action);
    }
}
