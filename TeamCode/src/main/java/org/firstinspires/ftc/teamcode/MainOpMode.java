package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogicStateMachine;

@TeleOp(name = ".pipers \uD83C\uDF36", group = "mainOp")
public class MainOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
	    Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);

        while (opModeInInit()){
            Elevator.update();
            Arm.update();
        }

        while (isStarted() && !isStopRequested()){

            Elevator.update();
            Arm.update();
        }


    }
}
