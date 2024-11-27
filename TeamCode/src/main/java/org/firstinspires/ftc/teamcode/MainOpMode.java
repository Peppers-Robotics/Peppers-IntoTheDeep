package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

@TeleOp(name = ".pipers \uD83C\uDF36", group = "mainOp")
public class MainOpMode extends LinearOpMode {

    public static boolean isClimbing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.telemetry = telemetry;
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);

        IntakeController.Initialize(gamepad1, gamepad2);

        Initialization.hubs.get(0).setConstant(0xff0000);
        Initialization.hubs.get(1).setConstant(0xff0000);

        Extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE);
        Arm.setArmAngle(0);
//        Claw.open();
//        Climb.disengagePTO();
//        Climb.PutDown();
        Initialization.Team = Initialization.AllianceColor.RED;
        Extendo.DropDown(0);
        while (opModeInInit()){
            Initialization.updateCacheing();
            Claw.open();
            Elevator.update();
            Extendo.update();
            Arm.update();
            Initialization.telemetry.update();
        }
        ElapsedTime time = new ElapsedTime();
        Claw.open();
        while (opModeIsActive()){

            Initialization.updateCacheing();
            if(Controls.Climbing){
                isClimbing = true;
                Controls.Climbing = false;
            }
            if(isClimbing){
                Climb.Update();
                continue;
            }
            Chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.left_trigger - gamepad1.right_trigger);

            Controls.Update();
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update();


            telemetry.addData("htz", 1/time.seconds());
            telemetry.addData("storage state", Storage.getStorageStatus().toString());
            telemetry.addData("Outtake state", OutTakeStateMachine.CurrentState.toString());
            telemetry.addData("intake state", IntakeController.CurrentState.toString());
            time.reset();
            telemetry.update();
        }
    }
}

