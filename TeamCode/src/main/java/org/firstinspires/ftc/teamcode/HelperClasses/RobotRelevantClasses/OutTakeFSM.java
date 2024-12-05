package org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses;

import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.json.JSONException;
import org.json.JSONObject;

public class OutTakeFSM {
    private static JSONObject table;
    static{
//        try {
            String json;
//            File f = new File("/sdcard/FIRST/xml/fsm_outtake.json");
//            Scanner s = new Scanner(f);
//            table = new JSONObject(s.toString());
//        } catch (JSONException | FileNotFoundException e) {
//            throw new RuntimeException(e);
//        }
    }

    public static OutTakeStateMachine.OutTakeStates getNextState(OutTakeStateMachine.OutTakeStates s, OutTakeStateMachine.OutTakeActions a){
        try {
            JSONObject thisState = table.getJSONObject(s.toString());
            JSONObject nextState = thisState.getJSONObject(a.toString());
            return OutTakeStateMachine.OutTakeStates.valueOf(nextState.toString());
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
    }
}
