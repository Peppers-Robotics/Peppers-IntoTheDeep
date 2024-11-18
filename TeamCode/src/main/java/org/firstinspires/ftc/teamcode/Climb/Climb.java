package org.firstinspires.ftc.teamcode.Climb;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.util.concurrent.ExecutionException;

public class Climb {
    public static ServoPlus W1, W2, PTO;
    public static double raise1 = 0, raise2 = 0, down1 = 0, down2 = 0;
    public static double PTOEngage = 0, PTOdisengage = 0;
    public static void Raise(){
        W1.setAngle(raise1);
        W2.setAngle(raise2);
    }
    public static void PutDown(){
        W1.setAngle(down1);
        W1.setAngle(down2);
    }

    public static void engagePTO(){
        PTO.setAngle(PTOEngage);
    }
    public static void disengagePTO(){
        PTO.setAngle(PTOdisengage);
    }

    enum states{
        GND_TO1,
        ONE_TWO,
        KEEP
    }
    public static states State;
    static {
        State = states.GND_TO1;
        isPrepared = false;
    }
    public static boolean isPrepared = false;

    private static void prepareForClimb(){
        engagePTO();
        Raise();

        isPrepared = true;
    }
    public static double BAR1 = 0, BAR2 = 0;

    public static void ClimbRobot(){
        if(!isPrepared) prepareForClimb();
        switch (State){
            case GND_TO1:
                if(Elevator.getTargetPosition() == 0)
                Elevator.setTargetPosition(BAR1);
                if(Elevator.ReachedTargetPosition())
                break;
        }
    }

    /*

    1. raise to lvl `x`
    1.5. wait 0.1s
    2. raise to lvl 0
    3. ExtendExtendo to max lenght
    4. raise to `x2`
    4.5. wait 0.1s
    5. raise to 0


 */

}
