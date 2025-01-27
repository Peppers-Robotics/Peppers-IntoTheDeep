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
        IDLE
    }
    public static States state = States.RETRACT;
    public static ElapsedTime time = new ElapsedTime();
    public static boolean wasDriverActivated = false;
    private static boolean reset = false;
    public static void update(){
        if(Controls.Grab && OutTakeLogic.CurrentState == OutTakeLogic.States.IDLE) {
            state = States.RETRACT;
            Controls.Grab = false;
        }
        switch (state){
            case IDLE:
                // :)
                Extendo.DISABLE = false;
                Extendo.Extend((int) (Extendo.getTargetPosition() - 35 * gamepad1.right_stick_y));
                Extendo.update();
                Robot.telemetry.addData("tp", Extendo.getTargetPosition());
                break;
            case RETRACT:
                Extendo.DISABLE = true;
                DropDown.setDown(0);

                ActiveIntake.Reverse(0.6);
                ActiveIntake.Block();

                Extendo.motor.setPower(-1);
                if((Extendo.motor.getCurrent(CurrentUnit.AMPS) >= 7.5 && Extendo.motor.getVelocity() < 2) || reset){
                    Extendo.motor.setPower(0);
                    reset = true;
                    if(time.seconds() > 0.1){
                        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Extendo.Extend(0);
                        state = States.IDLE;
                        Extendo.DISABLE = false;
                        ActiveIntake.powerOff();
                        reset = false;
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
            ActiveIntake.powerOn(1);
            DropDown.setDown(gamepad2.right_trigger);
            //unblock
            ActiveIntake.Unblock();
            wasDriverActivated = true;
        }
        if(Math.abs(gamepad2.left_trigger) > 0.02 && (ActiveIntake.isOff() || wasDriverActivated)){
            ActiveIntake.Reverse(gamepad2.left_trigger);
            //unblock
            ActiveIntake.Unblock();
            wasDriverActivated = true;
        }
        gamepad1.update();
        gamepad2.update();
        Robot.telemetry.addData("Intake State", state.toString());
    }
}
