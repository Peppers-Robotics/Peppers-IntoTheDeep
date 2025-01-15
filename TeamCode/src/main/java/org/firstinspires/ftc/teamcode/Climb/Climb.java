package org.firstinspires.ftc.teamcode.Climb;

import android.text.format.Time;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

import java.security.interfaces.ECKey;

@Config
public class Climb {
    public static ServoPlus W1, W2, PTO, PTO2;
    public static double down1 = 195, down2 = 245, raise1 = 305, raise2 = 115;
    public static double Kp = 0.02;
    public static double PTOEngage = 130, PTOdisengage = 180;
    public static double PTO2Engage = 200, PTO2Disengage = 170;
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
        PTO2.setAngle(PTO2Engage);
    }
    public static void disengagePTO(){
        PTO.setAngle(PTOdisengage);
        PTO2.setAngle(PTO2Disengage);
    }
    public static void initDisengagePTO(){
        PTO.setAngle(PTOdisengage);
        PTO2.setAngle(PTO2Disengage);
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
    public static double BAR1 = 550, BAR2 = 900;
    public static int action = 0;
    public static ElapsedTime TimeSinceLastStateChange = new ElapsedTime();
    public static long last_update = 0;
    public static void ChangeState(States s){
        State = s;
        TimeSinceLastStateChange.reset();
    }

    public static void Update(){
        Elevator.Disable = true;

//        Elevator.motor.setPower(Controls.gamepad2.right_stick_y);
        Chassis.BL.setPower(Controls.gamepad2.right_stick_y);
        Chassis.BR.setPower(Controls.gamepad2.right_stick_y);
        Extendo.pidEnable = false;
        Extendo.motor.setPower(Controls.gamepad1.right_stick_y);

    }

    public static void UpdateAuto(){
        Extendo.motor.setPower(-0.4);
        switch (State){
            case TILT_ROBOT:
                Raise();
                disengagePTO();
                Elevator.setTargetPosition(BAR1);
//                last_update = System.currentTimeMillis();
                ChangeState(States.PULL_UP);
                break;
            case PULL_UP:
                if(TimeSinceLastStateChange.seconds() < 0.5 && Elevator.getCurrentPosition() < BAR1 - 20) break;
                Elevator.setTargetPosition(0);
                if(TimeSinceLastStateChange.seconds() < 0.5 + 0.2) break;
                engagePTO();
                Elevator.setTargetPosition(-50);
                if(Elevator.getCurrentPosition() < 3 || TimeSinceLastStateChange.seconds() > 0.5 + 0.2 + 3){
                    Elevator.setTargetPosition(0);
                    PutDown();
                    disengagePTO();
                    Arm.setArmAngle(OutTakeStateMachine.IdleArmAngle);
                    Chassis.BL.setPower(0);
                    Chassis.BR.setPower(0);
                    ChangeState(States.EXTEND_EXTENDO_TO_MAX_AND_ETC2);
                }
                break;
            case EXTEND_EXTENDO_TO_MAX_AND_ETC2:
//                if(System.currentTimeMillis() - last_update < 500) break;
//                if(TimeSinceLastStateChange.seconds() < 0.1) break;
//                disengagePTO();
//                if(TimeSinceLastStateChange.seconds() < 0.1 + 0.1) break;
                Elevator.setTargetPosition(BAR2);
                if(Math.abs(Elevator.getCurrentPosition() - BAR2) <= 30) {
                    Arm.setArmAngle(OutTakeStateMachine.ArmTakeSpecimen + 40);
                    engagePTO();
                    ChangeState(States.PULL_ELEVATOR_A_BIT);
                }
                break;
            case PULL_ELEVATOR_A_BIT:
                if(TimeSinceLastStateChange.seconds() < 0.5) {  break;}
                Elevator.setTargetPosition(BAR2 - 150);
                engagePTO();
                if(Initialization.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) > -80) Elevator.Disable = true;
                else Elevator.Disable = false;
                if(TimeSinceLastStateChange.seconds() < 1 && Elevator.getCurrentPosition() > BAR2 - 40) break;
                ChangeState(States.IDLE_WHILE_UP);
                break;
            case IDLE_WHILE_UP:
                Elevator.setTargetPosition(-20);
//                if(Elevator.getCurrentPosition() < 0) Elevator.setTargetPosition(0);
                break;
        }
        Elevator.update();
        IntakeController.Update();
        Arm.update();
        Initialization.telemetry.addData("Climb State: ", Climb.State.toString());
        Initialization.telemetry.addData("Time", System.currentTimeMillis());
        Initialization.telemetry.addData("Time2", TimeSinceLastStateChange.seconds());
        Initialization.telemetry.addData("Pitch", Initialization.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
    }
}
