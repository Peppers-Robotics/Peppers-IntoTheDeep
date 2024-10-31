package org.firstinspires.ftc.teamcode.OutTake;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;

public class OutTakeController extends GenericController {

    private static boolean ScoreSpecimen = false;

    public static void update() throws Exception {
        if(Controls.Grab){
            if(Claw.isClosed()) Claw.open();
            else Claw.close();
            Controls.Grab= false;
        }
        if(Controls.GrabSpecimen){
            OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.GRAB_SPECIMEN);

            Controls.GrabSpecimen = false;
            ScoreSpecimen = true;
        }
        if(Controls.ScoreLevel1){
            if(ScoreSpecimen) {
                OutTakeLogicStateMachine.ScoreSpecimen(1);
                OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.SCORE_SPECIMEN);
            } else {
                OutTakeLogicStateMachine.ScoreSample(1);
                OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.SCORE_SAMPLE);
            }

            Controls.ScoreLevel1 = false;
        }
        else if(Controls.ScoreLevel2){
            if(ScoreSpecimen) {
                OutTakeLogicStateMachine.ScoreSpecimen(2);
                OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.SCORE_SPECIMEN);
            } else {
                OutTakeLogicStateMachine.ScoreSample(2);
                OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.SCORE_SAMPLE);
            }

            Controls.ScoreLevel2 = false;
        }

        if(Controls.DunkToScore && ScoreSpecimen){
            OutTakeLogicStateMachine.SetState(OutTakeLogicStateMachine.States.DUNK_TO_SCORE);
        }
    }
}
