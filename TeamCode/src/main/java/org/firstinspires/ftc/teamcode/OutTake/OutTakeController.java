package org.firstinspires.ftc.teamcode.OutTake;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;
import org.firstinspires.ftc.teamcode.Initialization;

import java.util.concurrent.RecursiveTask;

public class OutTakeController extends GenericController {
    public static boolean ScoreSpecimen = false;
    public static void Update(){
        if(Controls.Grab){
            if(Claw.isClosed()) {
                Claw.open();
                ScoreSpecimen = false;
            }
            else Claw.close();

            Controls.Grab = false;
        }
        if(Controls.GrabSpecimen){
            ScoreSpecimen = true;
            OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.EXTEND_TO_TAKE_SPECIMEN);
            Controls.GrabSpecimen = false;
            Claw.open();
        }
        if(Controls.ScoreLevel1){
            if(ScoreSpecimen){
                OutTakeLogicStateMachine.ElevatorScoreSpecimen = OutTakeLogicStateMachine.ElevatorSpecimen1;
                OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.EXTEND_TO_SCORE_SPECIMEN);
            } else {
                OutTakeLogicStateMachine.ElevatorScoreSample = OutTakeLogicStateMachine.ElevatorSample1;
                OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.EXTEND_TO_SCORE_SAMPLE);
            }
            Controls.ScoreLevel1 = false;

        } else if(Controls.ScoreLevel2){
            if(ScoreSpecimen){
                OutTakeLogicStateMachine.ElevatorScoreSpecimen = OutTakeLogicStateMachine.ElevatorSpecimen2;
                OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.EXTEND_TO_SCORE_SPECIMEN);
            } else {
                OutTakeLogicStateMachine.ElevatorScoreSample = OutTakeLogicStateMachine.ElevatorSample2;
                OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.EXTEND_TO_SCORE_SAMPLE);
            }
            Controls.ScoreLevel2 = false;
        } else if(Controls.Retract){
            OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.RETRACT);

            Controls.Retract = false;
        } else if(Controls.DunkToScore){
            OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.PUSH_SPECIMEN);

            Controls.DunkToScore = false;
        }
//        if(-gamepad2.right_stick_y <= -0.5 && OutTakeLogicStateMachine.canDunk) OutTakeLogicStateMachine.ChangeState(OutTakeLogicStateMachine.States.PUSH_SPECIMEN);
//        else Arm.setArmAngle(OutTakeLogicStateMachine.ArmTakeSpecimen);
        OutTakeLogicStateMachine.Update();
    }
}
