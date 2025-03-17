package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
            Robot.telemetry.addData("elevator current motor", Elevator.motor.getCurrent(CurrentUnit.AMPS));
            Robot.telemetry.addData("elevator current motor2", Elevator.motor2.getCurrent(CurrentUnit.AMPS));
            Elevator.update();
            Robot.clearCache();
        }
    }
}
