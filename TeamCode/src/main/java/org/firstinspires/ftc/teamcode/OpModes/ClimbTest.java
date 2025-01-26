package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
@TeleOp
public class ClimbTest extends LinearOpMode {
    public static boolean climb = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Arm.setArmAngle(13);
        Climb.DisengagePTO();
        Climb.DeactivateWheelie();
        DropDown.setDown(0);

        Climb.run = Climb.climb.clone();
        Elevator.setTargetPosition(0);


        waitForStart();

        while (opModeIsActive()){
            Robot.clearCache();
            if(climb){
                Climb.Update();
            }

            Elevator.update();
            Arm.update();
        }
    }
}
