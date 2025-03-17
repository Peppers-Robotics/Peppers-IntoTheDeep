package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Prezentare")
@Config
public class Prezentare extends LinearOpMode {
    public static double elevatorPos = 0, extendoPos = 0, armAngle = 0, retraction = 0, dropDown = 0;
    public static boolean claw = false;
    @Override
    public void runOpMode() {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();

        waitForStart();

        while (opModeIsActive()) {
            Robot.clearCache();
            Elevator.setTargetPosition(elevatorPos);
            Extendo.Extend((int) extendoPos);
            Arm.setArmAngle(armAngle);
            Extension.Extend(retraction);
            DropDown.setDown(dropDown);
            if(claw){
                Claw.close();
            } else {
                Claw.open();
            }

            Elevator.update();
            Extendo.update();
            Arm.update();
        }
    }
}
