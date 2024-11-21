package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDPatternCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        Initialization.hubs.get(0).setConstant(0xff0000);
        Initialization.hubs.get(1).setConstant(0xff0000);

        OutTakeLogicStateMachine.CurrentState = OutTakeLogicStateMachine.States.IDLE;
        Elevator.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setArmAngle(0);
        Claw.open();
        Initialization.Team = Initialization.AllianceColor.RED;
        while (opModeInInit()){
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
        }
        ElapsedTime time = new ElapsedTime();
        Claw.open();
        while (opModeIsActive()){

            Initialization.updateCacheing();
            Controls.Update();

            Chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.left_trigger - gamepad1.right_trigger);

            IntakeController.Update();
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
            Extendo.update();

            telemetry.addData("Outtake state", OutTakeLogicStateMachine.CurrentState.name());
            telemetry.addData("Claw state", Claw.isClosed() ? "open" : "closed");
            telemetry.addData("Claw color", Claw.clawSensor.getColorSeenBySensor().name());
            telemetry.addData("Claw distance", Claw.clawSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Claw pos", Claw.clawServo.getAngle());
            telemetry.addData("htz", 1/time.seconds());
            telemetry.addData("storage state", Storage.getStorageStatus().toString());
            time.reset();
            telemetry.update();
        }
    }
}

