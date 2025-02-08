package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class IntakeLogic extends GenericController {
    public enum States {
        RETRACT,
        IDLE,
        RESET
    }
    public static States state = States.RETRACT;
    public static ElapsedTime time = new ElapsedTime(), blocker = new ElapsedTime(), veloTimer = new ElapsedTime();
    public static boolean wasDriverActivated = false;
    private static boolean reset = false;
    private static int pos = 0;
    public static void update(){
        if(Controls.ImogenDriver) {
//            gamepad2.left_trigger = -Math.min(0, gamepad1.right_stick_x);
//            gamepad2.right_trigger = Math.max(0, gamepad1.right_stick_x);
            gamepad2.left_trigger = gamepad1.gamepad.left_bumper ? 1 : 0;
            gamepad2.right_trigger = gamepad1.gamepad.right_bumper ? 1 : 0;
        }
        if(Controls.RetractExtendo && OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE) {
            state = States.RETRACT;
            Controls.RetractExtendo = false;
        }
        switch (state){
            case IDLE:
                // :)
                if(Math.abs(gamepad1.right_stick_y) > 0.01) {
                    Extendo.DISABLE = true;
                    Extendo.PowerOnToTransfer = false;
//                Extendo.Extend((int) (Extendo.getTargetPosition() + 35 * (gamepad1.right_stick_y * gamepad1.right_stick_y)));
//                Extendo.update();
                    if (Extendo.getCurrentPosition() < 0 && gamepad1.right_stick_y > 0) break;
                    if (Extendo.getCurrentPosition() > Extendo.getMaxPosition() && gamepad1.right_stick_y < 0)
                        break;
                    Extendo.motor.setPower(-Math.signum(gamepad1.right_stick_y) * (gamepad1.right_stick_y * gamepad1.right_stick_y));
                    pos = Extendo.getCurrentPosition();
                } else {
                    Extendo.DISABLE = false;
                    Extendo.Extend(pos);
                }
                Robot.telemetry.addData("tp", Extendo.getTargetPosition());
                break;
            case RETRACT:
                Extendo.DISABLE = true;
                DropDown.setDown(0);

                ActiveIntake.Reverse(0.6);
                ActiveIntake.Block();

                Extendo.motor.setPower(-1);
                if(Math.abs(Extendo.motor.getVelocity()) > 2 && Extendo.motor.getCurrent(CurrentUnit.AMPS) <= 4) veloTimer.reset();
                if(reset || veloTimer.seconds() > 0.4){
                    Extendo.motor.setPower(0);
                    ActiveIntake.powerOff();
                    reset = true;
                    if(time.seconds() > 0.1){
                        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Extendo.Extend(0);
                        pos = 0;
                        state = States.IDLE;
                        Extendo.DISABLE = false;
                        ActiveIntake.powerOff();
                        ActiveIntake.Block();
                        reset = false;
                        if(Storage.hasTeamPice())
                            Controls.Transfer = true;
                    }
                } else time.reset();
                break;
            case RESET: // unused
                Extendo.DISABLE = true;
                Extendo.motor.setPower(-1);
                DropDown.setDown(0);
                if((Extendo.motor.getCurrent(CurrentUnit.AMPS) >= 6.5 && Math.abs(Extendo.motor.getVelocity()) <= 1) || reset){ // STALL
                    reset = true;
                    Extendo.motor.setPower(0);
                    if(time.seconds() >= 0.1){
                        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Extendo.Extend(0);
                        pos = 0;
                        state = States.IDLE;
                        reset = false;
                        Extendo.DISABLE = false;
                        if(Storage.hasTeamPice())
                            Controls.Transfer = true;
                    }
                } else time.reset();
                break;

        }
        if(wasDriverActivated && !ActiveIntake.isOff() && Math.abs(gamepad2.right_trigger) <= 0.02 && Math.abs(gamepad2.left_trigger) <= 0.02){
            ActiveIntake.powerOff();
            DropDown.setDown(0);
            //block
            ActiveIntake.Block();
            wasDriverActivated = false;
        }
        if(Math.abs(gamepad2.right_trigger) > 0.02 && (ActiveIntake.isOff() || wasDriverActivated)){
            if(Storage.hasTeamPice()) {
                if(blocker.seconds() >= 0.15) {
                    ActiveIntake.Block();
                    ActiveIntake.Reverse(0.7);
                    DropDown.setDown(0);
                }
            } else {
                blocker.reset();
                ActiveIntake.powerOn(1);
                ActiveIntake.Unblock();
                DropDown.setDown(gamepad2.right_trigger);
            }
            //unblock
            wasDriverActivated = true;
        }
        if(Math.abs(gamepad2.left_trigger) > 0.02 && (ActiveIntake.isOff() || wasDriverActivated)){
            ActiveIntake.Reverse(gamepad2.left_trigger);
            //unblock
            ActiveIntake.Unblock();
            wasDriverActivated = true;
        }
        gamepad1.update();
        if(!Controls.ImogenDriver) {
            gamepad2.update();
        }
        Robot.telemetry.addData("Intake State", state.toString());
    }
}
