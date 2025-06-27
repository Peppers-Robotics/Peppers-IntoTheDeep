package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;
import org.firstinspires.ftc.teamcode.OpModes.OpModeManager;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class IntakeLogic extends GenericController {
    public enum States {
        RETRACT,
        IDLE,
        RESET
    }
    public static States state = States.IDLE;
    public static ElapsedTime time = new ElapsedTime(), blocker = new ElapsedTime(), time2 = new ElapsedTime(), reverseTimer = new ElapsedTime();
    public static boolean wasDriverActivated = false;
    private static boolean Reverse = false;
    public static boolean IgnoreUntilNext = false;
    public static int offset = 0;
    public static void update(){
        if(Controls.ResetExtendoD2){
            if(!Extendo.lm.getState()) Extendo.motor.setPower(-1);
            else {
                Extendo.motor.setPower(0);
                Controls.ResetExtendoD2 = false;
            }
        }
//        if(gamepad2.left_trigger <= 0.05)
            gamepad2.left_trigger = gamepad1.gamepad.left_bumper ? 1 : 0;
//        if(gamepad1.right_trigger <= 0.05)
            gamepad2.right_trigger = gamepad1.gamepad.right_bumper ? 1 : 0;

        if((gamepad1.wasPressed.square || gamepad2.wasPressed.square) && OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE){
            Controls.RetractExtendo = false;
            state = States.RETRACT;
        }
        if(Controls.RetractExtendo && (OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE) && Storage.hasTeamPice()) {
            state = States.RETRACT;
            Controls.RetractExtendo = false;
        }
        if(OutTakeLogic.Transfering) gamepad1.right_stick_y = 0;
        if(Extendo.getCurrentPosition() <= 80) Extendo.DISABLE = false;
        else Extendo.DISABLE = true;
        switch (state){
            case IDLE:
                if(OutTakeLogic.Transfering) break;
                // :)
                if(Math.abs(gamepad1.right_stick_y) > 0.01) {
                    Extendo.PowerOnToTransfer = false;
                    Extendo.DISABLE = true;
//                Extendo.Extend((int) (Extendo.getTargetPosition() + 35 * (gamepad1.right_stick_y * gamepad1.right_stick_y)));
//                Extendo.update();
                    if (Extendo.getCurrentPosition() < 10 && gamepad1.right_stick_y > 0){
                        Extendo.motor.setPower(0);
                        break;
                    }
                    if (Extendo.getCurrentPosition() > Extendo.getMaxPosition() && gamepad1.right_stick_y < 0) {
                        Extendo.motor.setPower(0);
                        break;
                    }
                    Extendo.motor.setPower(-Math.signum(gamepad1.right_stick_y) * (gamepad1.right_stick_y * gamepad1.right_stick_y));
                } else if(!Controls.ResetExtendoD2) {
                    Extendo.motor.setPower(0);
                }
                if(Storage.isStorageEmpty() || Storage.getStorageStatus() == Storage.SpecimenType.YELLOW) Controls.Throw = false;
                if(Storage.hasTeamPice() && Controls.Throw && OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE_TAKE_SPECIMEN){
                    state = States.RETRACT;
                    Elevator.setTargetPosition(150);
                    Arm.setArmAngle(60);
                    Claw.openWide();
//                    Arm.setArmAngle(OutTakeLogic.ArmTransfer);
//                    Arm.setArmAngle(OutTakeLogic.ArmTransfer - 3);
                }
                if(Storage.hasTeamPice() && !IgnoreUntilNext){
                    Controls.RetractExtendo = true;
                }
                if(!Storage.hasTeamPice() && IgnoreUntilNext){
                    IgnoreUntilNext = false;
                }
                Robot.telemetry.addData("tp", Extendo.getCurrentPosition());
                time2.reset();
                break;
            case RETRACT:
                Extendo.DISABLE = true;
                DropDown.setDown(0);
                if(Controls.Throw && Elevator.getCurrentPosition() >= 130){
                    Controls.Throw = false;
                    Arm.setArmAngle(OutTakeLogic.ArmTransfer);
//                    Arm.armProfile.setInstant(OutTakeLogic.ArmTransfer);
                }
                if(time2.seconds() >= 0.1)
                    ActiveIntake.Reverse(0.6);
                ActiveIntake.Block();

                Extendo.motor.setPower(-1);
//                if(Math.abs(Extendo.motor.getVelocity()) > 2 && Extendo.motor.getCurrent(CurrentUnit.AMPS) <= 4) veloTimer.reset();
                if(Extendo.lm.getState()){
                    Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ActiveIntake.powerOff();
                    Extendo.Extend(0);
                    Extendo.motor.setPower(0);
                    if(Storage.hasTeamPice())
                        Controls.Transfer = true;
                    state = States.IDLE;
                }
                break;

        }
        if(Math.abs(gamepad2.left_trigger) <= 0.05 && Math.abs(gamepad2.right_trigger) <= 0.05 && wasDriverActivated){
            DropDown.setDown(0);
            ActiveIntake.powerOff();
            if(Storage.hasTeamPice()){
                ActiveIntake.Block();
            } else ActiveIntake.Unblock();
            DropDown.setDown(0);
            wasDriverActivated = false;
        }
        if (Math.abs(gamepad2.left_trigger) > 0.05) {
            ActiveIntake.Reverse(0.7);
            DropDown.setDown(0);
            wasDriverActivated = true;
            ActiveIntake.Unblock();
        } else if (gamepad2.right_trigger > 0.05) {
            if (Storage.hasTeamPice()) {
                ActiveIntake.Block();
                DropDown.setDown(0);
                if(blocker.seconds() >= 0.1) {
                   ActiveIntake.Reverse(0.7);
                   Reverse = true;
                }
            } else {
                blocker.reset();
                ActiveIntake.powerOn(1);
                DropDown.setDown(gamepad2.right_trigger);
            }
            wasDriverActivated = true;
        }
        /*if(Storage.hasTeamPice() && Reverse){
            if(reverseTimer.seconds() <= 1) ActiveIntake.Reverse(0.7);
            else {
                ActiveIntake.powerOff();
                Reverse = false;
            }
        } else reverseTimer.reset();*/

        gamepad1.update();
        if(!Controls.ImogenDriver) {
            gamepad2.update();
        }
        Robot.telemetry.addData("Intake State", state.toString());
//        RobotLog.ii("Intake State", state.toString());
    }
}
