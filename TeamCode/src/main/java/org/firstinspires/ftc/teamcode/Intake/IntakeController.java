package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

public class IntakeController extends GenericController {
    public static int Resolution;

    static {
        Resolution = 20;
    }

    public enum IntakeStates{
        IDLE_RETRACTED,
        RETRACT_EXTENDO,
        IDLE_EXTENDED,
    }
    public static IntakeStates CurrentState = IntakeStates.RETRACT_EXTENDO;
    public static ElapsedTime TimeSinceStateStartedRunning = new ElapsedTime();
    public static void ChangeState(IntakeStates state){
        CurrentState = state;
        TimeSinceStateStartedRunning.reset();
    }

    public static void Update() {
        switch (CurrentState){
            case RETRACT_EXTENDO:
                Extendo.DropDown(0);
                if(!Extendo.DropDownProfile.motionEnded()) break;

                Extendo.motor.setPower(1);

                if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                if(Math.abs(Extendo.motor.getVelocity()) > 5) break;

                Extendo.motor.setPower(0);
                Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Extendo.motor.setPower(0.5);

                if(TimeSinceStateStartedRunning.seconds() >= 0.2 + 0.2){
                    if(Storage.hasAlliancePice())
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE_WITH_SAMPLE);
                    ChangeState(IntakeStates.IDLE_RETRACTED);
                }

                break;
            case IDLE_RETRACTED:
                Extendo.motor.setPower(0.8);
                if(Extendo.getCurrentPosition() > 50) Extendo.motor.setMotorDisable();
                else Extendo.motor.setMotorEnable();
                if(gamepad1.left_stick_y != 0){
                    ChangeState(IntakeStates.IDLE_EXTENDED);
                }
                break;
            case IDLE_EXTENDED:
                Extendo.motor.setPower(gamepad1.left_stick_y);
                if(Storage.hasAlliancePice()) {
                    ChangeState(IntakeStates.RETRACT_EXTENDO);
                }
                break;
        }
    }
}
