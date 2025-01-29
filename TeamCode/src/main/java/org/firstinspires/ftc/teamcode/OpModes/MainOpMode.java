package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@TeleOp(name = ".mainopMode")
@Disabled
public class MainOpMode extends LinearOpMode {
    public static boolean isClimbing = false;
    public static final double tSlow = 0.5, rotSlow = 0.4;
    public double rotSpeed = 0.8, tSpeed = 0.6;

    public static double getSquaredSigned(double h){
        return Math.signum(h) * (h * h);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);
        IntakeLogic.Initialize(gamepad1, gamepad2);
        isClimbing = false;

        IntakeLogic.state = IntakeLogic.States.RETRACT;

        waitForStart();
        Robot.enable();
        Claw.open();
        ActiveIntake.Block();
        DropDown.setDown(0);
        Extendo.pidController.setFreq(40);

        while (opModeIsActive()){
            Robot.clearCache();
            if(Controls.Climbing && !isClimbing){
                Climb.run = Climb.climb.clone();
                isClimbing = true;
                Controls.Climbing = false;
            }
            if(isClimbing){
                Climb.Update();
                break;
            }
            if(gamepad1.cross){
                tSpeed = 1;
                rotSpeed = 0.9;
            } else {
                tSpeed = tSlow;
                rotSpeed = rotSlow;
            }
//            Chassis.drive(gamepad1.left_stick_x * tSpeed, -gamepad1.left_stick_y * tSpeed, (gamepad1.right_trigger - gamepad1.left_trigger) * rotSpeed);
            Chassis.drive(getSquaredSigned(gamepad1.left_stick_x), -getSquaredSigned(gamepad1.left_stick_y), getSquaredSigned(gamepad1.right_trigger - gamepad1.left_trigger));
            OutTakeLogic.update();
            IntakeLogic.update();
            Arm.update();
            Elevator.update();
            Extendo.update();
            Controls.Update();
        }
    }
}
