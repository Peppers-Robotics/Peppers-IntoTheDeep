package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class ElevatorPID extends LinearOpMode {
    public static double TargetPosition = 0;
    public static double freq = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeElevator();

        waitForStart();

        while (opModeIsActive()){

            Elevator.setTargetPosition(TargetPosition);
            Elevator.controller.setFreq(freq);
            Elevator.update();
            Robot.clearCache();
        }
    }
}
