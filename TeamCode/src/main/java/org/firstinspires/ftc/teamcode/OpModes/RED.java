package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = ".peppersRED \uD83D\uDFE5")
public class RED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeManager opmode = new OpModeManager(hardwareMap, gamepad1, gamepad2, telemetry, Storage.Team.RED);
        Chassis.Autonomous = false;

        waitForStart();
        Robot.enable();
        DropDown.setDown(0);
        while (opModeIsActive()){
            opmode.update();
            Robot.telemetry.addData("Lift elevator", Elevator.getCurrentPosition());
            Robot.telemetry.addData("color", Storage.getStorageStatus());
            Robot.telemetry.update();
        }
    }
}
