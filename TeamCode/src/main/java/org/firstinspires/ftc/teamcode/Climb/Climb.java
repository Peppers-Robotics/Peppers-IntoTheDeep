package org.firstinspires.ftc.teamcode.Climb;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.security.interfaces.ECKey;
import java.sql.Time;

@Config
public class Climb {
    public static ServoPlus W1, W2, PTO;
    public static double down1 = 135, down2 = 290, raise1 = 300, raise2 = 100;
    public static double Kp = 0.01;
    public static double PTOEngage = 232, PTOdisengage = 170;
    public static double PTO2Engage = 150, PTO2Disengage = 185;
    public static void Raise(){
        W1.setAngle(raise1);
        W2.setAngle(raise2);
    }
    public static void PutDown(){
        W1.setAngle(down1);
        W2.setAngle(down2);
    }

    public static boolean isPTOEngaged(){
        if(PTO == null) return false;
        return PTO.isEqualToAngle(PTOEngage);
    }

    public static void engagePTO(){
        PTO.setAngle(PTOEngage);
    }
    public static void disengagePTO(){
        PTO.setAngle(PTOdisengage);
    }

    public enum States{
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
    public static double BAR1 = 300, BAR2 = 700;
    public static int action = 0;
    public static ElapsedTime TimeSinceLastStateChange = new ElapsedTime();
    public static NanoClock ClimbTime = NanoClock.system();
    public static double last_update = 0;
    public static void ChangeState(States s){
        State = s;
        TimeSinceLastStateChange.reset();
    }

    public static void Update(){
        engagePTO();
        Elevator.Disable = true;

//        Elevator.motor.setPower(Controls.gamepad2.right_stick_y);
        Chassis.BL.setPower(Controls.gamepad2.right_stick_y);
        Chassis.BR.setPower(Controls.gamepad2.right_stick_y);
        Extendo.pidEnable = false;
        Extendo.motor.setPower(Controls.gamepad1.right_stick_y);

    }

    public static void UpdateAuto(){
        switch (State){
            case TILT_ROBOT:
                Raise();
                disengagePTO();
                Elevator.setTargetPosition(BAR1);
                ChangeState(States.PULL_UP);
                last_update = ClimbTime.seconds();
                break;
            case PULL_UP:
                if(ClimbTime.seconds() - last_update < 1) break;
                Elevator.setTargetPosition(0);
                if(ClimbTime.seconds() - last_update < 1.2) break;
                Elevator.Disable = true;
                engagePTO();
                if(Elevator.getCurrentPosition() > 10){
                    Chassis.BL.setPower(1);
                    Chassis.BR.setPower(1);
                } else {
                    Chassis.BL.setPower(0);
                    Chassis.BR.setPower(0);
                    if(action == 0) {
                        last_update = ClimbTime.seconds();
                        ChangeState(States.EXTEND_EXTENDO_TO_MAX_AND_ETC2);
                        action = 1;
                    }
                    else ChangeState(States.IDLE_WHILE_UP);
                }
                break;
            case EXTEND_EXTENDO_TO_MAX_AND_ETC2:
                if(ClimbTime.seconds() < 0.5) break;
                disengagePTO();
                Elevator.setTargetPosition(BAR2);
                if(Elevator.ReachedTargetPosition()) {
                    last_update = ClimbTime.seconds();
                    Arm.setArmAngle(300);
                    ChangeState(States.PULL_ELEVATOR_A_BIT);
                }
                break;
            case PULL_ELEVATOR_A_BIT:
                if(ClimbTime.seconds() < 0.5) break;
                if(Elevator.getCurrentPosition() > 10){
                    Chassis.BL.setPower(1);
                    Chassis.BR.setPower(1);
                }
                ChangeState(States.RETRACT_EXTENDO);
                break;
            case IDLE_WHILE_UP:
                Chassis.BL.setPower(Elevator.getCurrentPosition() * Kp);
                Chassis.BR.setPower(Elevator.getCurrentPosition() * Kp);
                break;
        }
        Elevator.update();
        IntakeController.Update();
        Arm.update();
        Initialization.telemetry.addData("Climb State: ", Climb.State.toString());
        Initialization.telemetry.addData("Time: ", ClimbTime.seconds() - last_update);
    }
}
