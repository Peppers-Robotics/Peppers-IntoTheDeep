package org.firstinspires.ftc.teamcode.TuneModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

@Config
@TeleOp(group = "PID tuner")
public class TuneClimbPID extends LinearOpMode {

    public static double targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeChassis();
        Initialization.initializeClimb();
        Initialization.initializeElevator();
        Elevator.RESET = false;

        waitForStart();

        while (opModeIsActive()){
            Initialization.updateCacheing();
            Climb.engagePTO();
            Climb.Raise();
            Elevator.setTargetPosition(targetPosition);

            Elevator.update();
            Initialization.telemetry.update();
        }

    }
}
