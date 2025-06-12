package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HelperClasses.Colors;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.GenericController;
import org.firstinspires.ftc.teamcode.OpModes.OpModeManager;
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
    public static ElapsedTime time = new ElapsedTime(), blocker = new ElapsedTime(), time2 = new ElapsedTime();
    public static boolean wasDriverActivated = false;
    private static int pos = 0;
    public static int offset = 0;
    public static void update(){
        if(true) {
//            gamepad2.left_trigger = -Math.min(0, gamepad1.right_stick_x);
//            gamepad2.right_trigger = Math.max(0, gamepad1.right_stick_x);
            gamepad2.left_trigger = gamepad1.gamepad.left_bumper ? 1 : 0;
            gamepad2.right_trigger = gamepad1.gamepad.right_bumper ? 1 : 0;
        }
        if(Controls.RetractExtendo && OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE && Storage.hasTeamPice()) {
            state = States.RETRACT;
            Controls.RetractExtendo = false;
        }
        else{
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
                    pos = Extendo.getCurrentPosition();
                } else {
                    Extendo.motor.setPower(0);
                }
                if(Storage.hasTeamPice() ){
                    Controls.RetractExtendo = true;
                }
                Robot.telemetry.addData("tp", Extendo.getCurrentPosition());
                time2.reset();
                break;
            case RETRACT:
                Extendo.DISABLE = true;
                DropDown.setDown(0);

                ActiveIntake.Reverse(0.6);
                ActiveIntake.Block();

                Extendo.motor.setPower(-1);
//                if(Math.abs(Extendo.motor.getVelocity()) > 2 && Extendo.motor.getCurrent(CurrentUnit.AMPS) <= 4) veloTimer.reset();
                if(!Extendo.lm.getState()){
                    Extendo.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Extendo.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ActiveIntake.powerOff();
                    Extendo.Extend(30);
                    Extendo.motor.setPower(0);
                    if(Storage.hasTeamPice() || Storage.getStorageStatus() == Storage.SpecimenType.YELLOW)
                        Controls.Transfer = true;
                    state = States.IDLE;
                }
                break;

        }

        if (Math.abs(gamepad2.left_trigger) <= 0.05 && Math.abs(gamepad2.right_trigger) <= 0.05 && wasDriverActivated) {
            ActiveIntake.powerOff();
            DropDown.setDown(0);
            wasDriverActivated = false;
            //block
            if (Storage.hasTeamPice()) {
                ActiveIntake.Block();
            } else
                ActiveIntake.Unblock();
        } else if (Math.abs(gamepad2.left_trigger) > 0.05 && Math.abs(gamepad2.right_trigger) > 0.05) {
            ActiveIntake.Reverse(0.8);
            ActiveIntake.Unblock();
            wasDriverActivated = true;
        } else if (gamepad2.right_trigger > 0.05) {
            if (Storage.hasTeamPice()) {
//               if(blocker.seconds() >= 0.15) {
                ActiveIntake.Block();

                DropDown.setDown(0);
//               }
            } else {
                ActiveIntake.powerOn(1);
                ActiveIntake.Unblock();
                DropDown.setDown(gamepad2.right_trigger);
            }
            wasDriverActivated = true;
        } else if (gamepad2.left_trigger > 0.05) {
            if (Storage.hasTeamPice() || Storage.sensor.getColorSeenBySensor() == Colors.ColorType.YELLOW) {
                ActiveIntake.Block();
                DropDown.setDown(0);
                wasDriverActivated = true;
            } else {
                ActiveIntake.Reverse(0.6);
                //unblock
                ActiveIntake.Unblock();
            }
            wasDriverActivated = true;
        }

        gamepad1.update();
        if(!Controls.ImogenDriver) {
            gamepad2.update();
        }
        Robot.telemetry.addData("Intake State", state.toString());
//        RobotLog.ii("Intake State", state.toString());
    }
}
