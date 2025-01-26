package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeLogic;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;

@TeleOp
@Disabled
public class MainOpMode extends LinearOpMode {
    public static boolean isClimbing = false;
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
        DropDown.setDown(0);

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
            Chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger);
            OutTakeLogic.update();
            IntakeLogic.update();
            Arm.update();
            Elevator.update();
            Extendo.update();
            Controls.Update();
        }
    }
}
