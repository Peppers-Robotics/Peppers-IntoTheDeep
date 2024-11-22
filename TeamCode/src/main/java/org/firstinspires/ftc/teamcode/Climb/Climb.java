package org.firstinspires.ftc.teamcode.Climb;

import android.telephony.mbms.MbmsErrors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.xmlpull.v1.XmlPullParser;

import java.util.concurrent.ExecutionException;

public class Climb {
    public static ServoPlus W1, W2, PTO;
    public static double down1 = 120, down2 = 300, raise1 = 310, raise2 = 100;
    public static double PTOEngage = 270, PTOdisengage = 170;
    public static void Raise(){
        W1.setAngle(raise1);
        W2.setAngle(raise2);
    }
    public static void PutDown(){
        W1.setAngle(down1);
        W2.setAngle(down2);
    }

    public static boolean isPTOEngaged(){
        return false;
//        return PTO.isEqualToAngle(PTOEngage);
    }

    public static void engagePTO(){
        PTO.setAngle(PTOEngage);
    }
    public static void disengagePTO(){
        PTO.setAngle(PTOdisengage);
    }

    enum states{
        GND_TO1,
        PULL,
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
    public static boolean Continue = true;
    public static ElapsedTime timer = new ElapsedTime();
    public static void Update(){
        if(!isPrepared) prepareForClimb();
        switch (State){
            case GND_TO1:
                Arm.setArmAngle(180);
                if(Arm.motionCompleted())
                    Raise();
                Elevator.setTargetPosition(BAR1);
                if(Elevator.ReachedTargetPosition() && Arm.motionCompleted()){
                    engagePTO();
                    if(timer.seconds() >= 0.1){
                        State = states.PULL;
                    }
                } else timer.reset();
                Elevator.update();
                break;
            case PULL:
                Elevator.setTargetPosition(0);
                if(Elevator.ReachedTargetPosition()){
                    Extendo.Extend(1300);
                    Extendo.pidEnable = true;
                    if(-Extendo.motor.getCurrentPosition() >= 1200){
                        if(Continue)
                            State = states.ONE_TWO;
                        else
                            State = states.KEEP;
                    }
                }
                break;
            case ONE_TWO:
                Elevator.setTargetPosition(BAR2);
                if(Elevator.ReachedTargetPosition()){
                    State = states.PULL;
                    Continue = false;
                }
                break;
            case KEEP:
                Extendo.Extend(0);
                Elevator.setTargetPosition(0);
                break;
        }

        Arm.update();

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
