package org.firstinspires.ftc.teamcode.TuneModes.OutTake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

@TeleOp
@Config
@Disabled
public class OutTakeFSM extends LinearOpMode {
    public static OutTakeStateMachine.OutTakeStates s = OutTakeStateMachine.OutTakeStates.IDLE;
    public static OutTakeStateMachine.OutTakeActions a = OutTakeStateMachine.OutTakeActions.NULL;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Next state", org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.OutTakeFSM.getNextState(s, a));
            telemetry.update();
        }
    }
}
