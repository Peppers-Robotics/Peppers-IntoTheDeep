package org.firstinspires.ftc.teamcode.Climb;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

public class Climb {
    public static ServoPlus W1, W2, PTO;
    public static double down1 = 135, down2 = 290, raise1 = 310, raise2 = 100;
    public static double PTOEngage = 270, PTOdisengage = 180;
    public static void Raise(){
        W1.setAngle(raise1);
        W2.setAngle(raise2);
    }
    public static void PutDown(){
        W1.setAngle(down1);
        W2.setAngle(down2);
    }

    public static boolean isPTOEngaged(){
        return PTO.isEqualToAngle(PTOEngage);
    }

    public static void engagePTO(){
        PTO.setAngle(PTOEngage);
    }
    public static void disengagePTO(){
        PTO.setAngle(PTOdisengage);
    }

    enum States{
        GET_ARM_UP,
        TILT_ROBOT,
        RAISE_ELEVATOR_TO_CLIMB1,
        PULL_UP,
        EXTEND_EXTENDO_TO_MAX_AND_ETC2,
        PULL_ELEVATOR_A_BIT,
        RETRACT_EXTENDO,
        IDLE_WHILE_UP
    }
    public static States State;
    static {
    }
    public static double BAR1 = 0, BAR2 = 0, BAR2hoverOn = 300, MaxExtendo = 1300;
    public static int action = 0;
    public static ElapsedTime TimeSinceLastStateChange = new ElapsedTime();
    public static void ChangeState(States s){
        State = s;
        TimeSinceLastStateChange.reset();
    }
    public static void Update(){
        switch (State){
            case GET_ARM_UP:
                Arm.setArmAngle(180);
                if(!Arm.motionCompleted()) break;
                ChangeState(States.TILT_ROBOT);
                break;
            case TILT_ROBOT:
                Raise();
                ChangeState(States.RAISE_ELEVATOR_TO_CLIMB1);
                break;
            case RAISE_ELEVATOR_TO_CLIMB1:
                disengagePTO();
                Elevator.setTargetPosition(BAR1);
                if(Elevator.getCurrentPosition() < BAR1 - 5) break;
                ChangeState(States.PULL_UP);
                break;
            case PULL_UP:
                engagePTO();
                if(TimeSinceLastStateChange.seconds() < 0.05) break;
                Elevator.setTargetPosition(0);
                if(Elevator.getCurrentPosition() > 10){
                    Chassis.BL.setPower(1);
                    Chassis.BR.setPower(1);
                } else {
                    Chassis.BL.setPower(0);
                    Chassis.BR.setPower(0);
                    disengagePTO();
                    if(action == 0) {
                        ChangeState(States.EXTEND_EXTENDO_TO_MAX_AND_ETC2);
                        action = 1;
                    }
                    else ChangeState(States.IDLE_WHILE_UP);
                }
                break;
            case EXTEND_EXTENDO_TO_MAX_AND_ETC2:
                IntakeController.gamepad1.right_stick_y = -1;
                Elevator.setTargetPosition(BAR2);
                if(Extendo.getCurrentPosition() < MaxExtendo - 150)
                    IntakeController.gamepad1.right_stick_y = 0;
                if (Extendo.motor.getPower() == 0 && Elevator.getCurrentPosition() >= BAR2) {
                    ChangeState(States.PULL_ELEVATOR_A_BIT);
                }
                break;
            case PULL_ELEVATOR_A_BIT:
                Elevator.setTargetPosition(BAR2hoverOn);
                if(!Elevator.ReachedTargetPosition()) break;
                ChangeState(States.RETRACT_EXTENDO);
                break;
            case RETRACT_EXTENDO:
                if(IntakeController.CurrentState == IntakeController.IntakeStates.IDLE_EXTENDED)
                    IntakeController.gamepad1.right_stick_y = 1;
                else {
                    IntakeController.gamepad1.right_stick_y = 0;
                    ChangeState(States.PULL_UP);
                }
                break;
            case IDLE_WHILE_UP:
                break;
        }
        Elevator.update();
        IntakeController.Update();
        Arm.update();
    }
}
