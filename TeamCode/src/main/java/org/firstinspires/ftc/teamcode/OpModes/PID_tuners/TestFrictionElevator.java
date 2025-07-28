package org.firstinspires.ftc.teamcode.OpModes.PID_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
@Config
public class TestFrictionElevator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Robot.InitializeHubs(hardwareMap);
        Robot.InitializeElevator();

        waitForStart();

        while (opModeIsActive()) {
            double kf = (Elevator.kfUp - Elevator.kfDown) / (Elevator.elevatorMax - Elevator.elevatorMin) * Elevator.getCurrentPosition();

            Elevator.motor.setPower(kf);
            Elevator.motor2.setPower(kf);
        }

        }
}
