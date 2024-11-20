package org.firstinspires.ftc.teamcode.TuneModes.OutTake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

@Config
@TeleOp(group = "Outtake")
public class ElevatorPIDTuner extends LinearOpMode {
    public static double TargetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.initializeElevator(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            Elevator.setTargetPosition(TargetPosition);
            Elevator.update();
            Initialization.updateCacheing();
//            telemetry.addData("Elevator pos", Elevator.getCurrentPosition());
            Initialization.telemetry.addData("Elevator targetPos", TargetPosition);
            Initialization.telemetry.update();
        }
    }
}
