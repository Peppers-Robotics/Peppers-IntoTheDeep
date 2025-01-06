package org.firstinspires.ftc.teamcode.Intake;

import android.text.format.Time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.MainOpModeRed;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

public class IntakeController extends GenericController {
    public static int Resolution;
    public static boolean isInAuto = false;

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
    public static boolean wasReseted = false, autoIntake = false;
    public static void ChangeState(IntakeStates state){
        CurrentState = state;
        TimeSinceStateStartedRunning.reset();
    }
    public static boolean optimization = true;
    public static void Update(){
        Update(false);
    }

    public static void Update(boolean isAuto) {
        if(Extendo.pidEnable){
            CurrentState = IntakeStates.IDLE_EXTENDED;
        }
        switch (CurrentState){
            case RETRACT_EXTENDO:
                DropDown.GoUp();
                if(!DropDown.isUp()) break;

                if(!wasReseted) {
                    Extendo.motor.setPower(0.5);
                    if(TimeSinceStateStartedRunning.seconds() < 0.2) break;
                    Extendo.motor.setPower(0);
                    if(TimeSinceStateStartedRunning.seconds() < 0.1 + 0.2) break;
                    Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Extendo.motor.setPower(1);
                    wasReseted = true;
                }
                if(Arm.getCurrentArmAngle() == OutTakeStateMachine.IdleArmAngle) {
                    if (TimeSinceStateStartedRunning.seconds() > 0.05) {
                        ActiveIntake.powerOn();
                    } else {
                        ActiveIntake.Reverse();
                    }
                }
                if(TimeSinceStateStartedRunning.seconds() >= 0.2 + 0.1 || !wasReseted){
                    if(Storage.hasAlliancePice() && OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE) {
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.TRANSFER_ARM);
                    }
                    ChangeState(IntakeStates.IDLE_RETRACTED);
                }

                break;
            case IDLE_RETRACTED:
                if(!Controls.ImogenDriver) {
                    if (gamepad2.right_trigger >= 0.05 || gamepad2.gamepad.right_bumper || gamepad2.left_trigger >= 0.1) {
                        if (gamepad2.gamepad.right_bumper) {
                            DropDown.GoMiddle();
                        } else if (gamepad2.right_trigger >= 0.05) {
                            DropDown.setInstantPosition(gamepad2.right_trigger);
                        } else if(gamepad2.left_trigger >= 0.12){
                            DropDown.GoUp();
                        }
                        ActiveIntake.powerOn();
                    } else if (gamepad2.gamepad.left_bumper || Storage.wrongPice()) {
                        ActiveIntake.Reverse();
                    } else {
                        ActiveIntake.powerOff();
                        DropDown.GoUp();
                    }
                } else {
                    if (gamepad1.right_stick_x > 0.15) {
                        DropDown.setInstantPosition(gamepad1.right_stick_x);
                        ActiveIntake.powerOn();
                    } else if (gamepad1.right_stick_x < -0.15) {
                        ActiveIntake.Reverse();
                    } else {
                        ActiveIntake.powerOff();
                        DropDown.GoUp();
                    }
                }


                if(DropDown.isUp() && Storage.hasAlliancePice()){
                    ChangeState(IntakeStates.RETRACT_EXTENDO);
                }
                Extendo.motor.setPower(1);
                if(Extendo.motor.getCurrentPosition() > -20 && optimization) {
                    Extendo.motor.setMotorDisable();
                }
                else {
                    Extendo.motor.setMotorEnable();
                }
                if(Math.abs(gamepad1.right_stick_y) >= 0.01){
                    Extendo.motor.setMotorEnable();
                    Extendo.motor.setPower(gamepad1.right_stick_y * (Controls.SlowDown ? 0.8 : 1));
                    ChangeState(IntakeStates.IDLE_EXTENDED);
                    wasReseted = false;
                }
                break;
            case IDLE_EXTENDED:
                if(!Controls.ImogenDriver) {
                    if (gamepad2.right_trigger >= 0.05 || gamepad2.gamepad.right_bumper || gamepad2.gamepad.left_trigger >= 0.1) {
                        if (gamepad2.gamepad.right_bumper) {
                            DropDown.GoMiddle();
                        } else if (gamepad2.right_trigger >= 0.05) {
                            DropDown.setInstantPosition(gamepad2.right_trigger);
                        } else if(gamepad2.left_trigger >= 0.12){
                            DropDown.GoUp();
                        }
                        ActiveIntake.powerOn();
                    } else if (gamepad2.gamepad.left_bumper || Storage.wrongPice()) {
                        ActiveIntake.Reverse();
                    } else {
                        ActiveIntake.powerOff();
                        DropDown.GoUp();
                    }
                } else {
                    if (gamepad1.right_stick_x > 0.15) {
                        DropDown.setInstantPosition(gamepad1.right_stick_x);
                        ActiveIntake.powerOn();
                    } else if (gamepad1.right_stick_x < -0.15) {
                        ActiveIntake.Reverse();
                    } else {
                        ActiveIntake.powerOff();
                        DropDown.GoUp();
                    }
                }
                if(!Extendo.pidEnable) {
                    Extendo.motor.setPower(gamepad1.right_stick_y);
                }
                if(Extendo.motor.getCurrentPosition() > -10 && gamepad1.right_stick_y > 0) {
                    ChangeState(IntakeStates.RETRACT_EXTENDO);
                    wasReseted = false;
                }
                break;

        }

        Initialization.telemetry.addData("Extendo current pos", Extendo.motor.getCurrentPosition());
        Initialization.telemetry.addData("extendo motor enabled", Extendo.motor.isMotorEnabled());
        Initialization.telemetry.addData("trigger", gamepad1.right_stick_y);
        if(!isAuto) {
            gamepad1.update();
            gamepad2.update();
        }
    }
}
