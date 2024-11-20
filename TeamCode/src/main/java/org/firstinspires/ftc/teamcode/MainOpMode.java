package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDPatternCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogicStateMachine;

import java.util.ArrayList;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

import javax.lang.model.element.ExecutableElement;

@TeleOp(name = ".pipers \uD83C\uDF36", group = "mainOp")
public class MainOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.telemetry = telemetry;
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);

        OutTakeController.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        Blinker.Step a = new Blinker.Step(0xff0000, 1, TimeUnit.DAYS);
        Vector<Blinker.Step> b = new Vector<>();
        b.add(a);
        Initialization.hubs.get(0).setPattern(b);
        OutTakeLogicStateMachine.CurrentState = OutTakeLogicStateMachine.States.IDLE;
        Elevator.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeInInit()){
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
        }

        while (opModeIsActive()){
            Initialization.updateCacheing();

            IntakeController.Update();
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
            Controls.Update();
            Extendo.update();

            telemetry.addData("Outtake state", OutTakeLogicStateMachine.CurrentState.name());
            telemetry.addData("Claw state", Claw.isClosed() ? "open" : "closed");
            telemetry.addData("Claw color", Claw.clawSensor.getColorSeenBySensor().name());
            telemetry.addData("Claw distance", Claw.clawSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
