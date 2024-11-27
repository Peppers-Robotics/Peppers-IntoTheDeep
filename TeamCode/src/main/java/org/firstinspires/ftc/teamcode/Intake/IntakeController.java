package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.GenericController;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.MainOpMode;
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

                Extendo.motor.setPower(0.4);

                if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                if(Extendo.motor.getVelocity() > 200) break;

                Extendo.motor.setPower(0);
                Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Extendo.motor.setPower(0.4);

                if(TimeSinceStateStartedRunning.seconds() >= 0.2 + 0.1){
                    if(Storage.hasAlliancePice() && OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE) {
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_IDLE_WITH_SAMPLE);
                    }
                    ChangeState(IntakeStates.IDLE_RETRACTED);
                }

                break;
            case IDLE_RETRACTED:
                if(gamepad2.right_trigger >= 0.1){
                    Extendo.DropDown(Extendo.MaxExtension);
                    ActiveIntake.powerOn();
                } else if(gamepad2.left_trigger >= 0.1){
                    ActiveIntake.Reverse();
                } else {
                    Extendo.DropDown(0);
                    ActiveIntake.powerOff();
                }
                if(Extendo.DropDownProfile.motionEnded() && Extendo.DropDownProfile.getTargetPosition() == 0 && Storage.hasAlliancePice()){
                    ChangeState(IntakeStates.RETRACT_EXTENDO);
                }
                Extendo.motor.setPower(1);
                if(Extendo.motor.getCurrentPosition() > -20) Extendo.motor.setMotorDisable();
                else Extendo.motor.setMotorEnable();
                if(gamepad1.right_stick_y != 0){
                    ChangeState(IntakeStates.IDLE_EXTENDED);
                    Extendo.motor.setMotorEnable();
                    Extendo.motor.setPower(gamepad1.right_stick_y);
                }
                break;
            case IDLE_EXTENDED:
                if(gamepad2.right_trigger >= 0.1){
                    Extendo.DropDown(Extendo.MaxExtension);
                    ActiveIntake.powerOn();
                } else if(gamepad2.left_trigger >= 0.1){
                    ActiveIntake.Reverse();
                } else {
                    Extendo.DropDown(0);
                    ActiveIntake.powerOff();
                }
                Extendo.motor.setPower(gamepad1.right_stick_y);
                if(Extendo.motor.getCurrentPosition() > -10 && gamepad1.right_stick_y > 0) {
                    ChangeState(IntakeStates.RETRACT_EXTENDO);
                }
                break;
        }
        Initialization.telemetry.addData("Extendo current pos", Extendo.motor.getCurrentPosition());
        Initialization.telemetry.addData("extendo motor enabled", Extendo.motor.isMotorEnabled());
        Initialization.telemetry.addData("trigger", gamepad1.right_stick_y);
        if(!MainOpMode.isClimbing)
            gamepad1.update();
        gamepad2.update();
    }
}
