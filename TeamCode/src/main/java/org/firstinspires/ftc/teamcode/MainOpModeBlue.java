package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;

@TeleOp(name = ".pipersBLUE \uD83C\uDF36", group = ".mainOp")
public class MainOpModeBlue extends LinearOpMode {

    public static boolean isClimbing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Initialization.telemetry = telemetry;
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);

        OutTakeStateMachine.inAuto = false;
        IntakeController.autoIntake = false;
        IntakeController.optimization = true;

        IntakeController.Initialize(gamepad1, gamepad2);

        Initialization.hubs.get(0).setConstant(0xff0000);
        Initialization.hubs.get(1).setConstant(0xff0000);

        gamepad2.setLedColor((double) 0xba, (double) 0x00, (double) 0x71, (int) 1e10);

        Extendo.pidEnable = false;
        IntakeController.ChangeState(IntakeController.IntakeStates.RETRACT_EXTENDO);
        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE);

        Initialization.Team = Initialization.AllianceColor.BLUE;

        DropDown.GoUp();
        Climb.PutDown();
        Climb.disengagePTO();

        Initialization.hubs.get(0).disengage();
        Initialization.hubs.get(1).disengage();

        while (opModeInInit()){
//            Initialization.updateCacheing();
//            Claw.open();
//            Elevator.update();
//            Extendo.update();
//            Arm.update();
            Initialization.telemetry.update();
        }

        Initialization.hubs.get(0).engage();
        Initialization.hubs.get(1).engage();

        ElapsedTime time = new ElapsedTime();
        Claw.open();
        ActiveIntake.UnblockIntake();

        while (opModeIsActive()){

            Initialization.updateCacheing();

            if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE){
                switch (Storage.getStorageStatus()) {
                    case RED:
                        Initialization.hubs.get(0).setConstant(0xff0000);
                        Initialization.hubs.get(1).setConstant(0xff0000);
                        break;
                    case BLUE:
                        Initialization.hubs.get(0).setConstant(0x0000ff);
                        Initialization.hubs.get(1).setConstant(0x0000ff);
                        break;
                    case YELLOW:
                        Initialization.hubs.get(0).setConstant(0xffff00);
                        Initialization.hubs.get(1).setConstant(0xffff00);
                        break;
                    case NONE:
                        Initialization.hubs.get(0).setConstant(0xffffff);
                        Initialization.hubs.get(1).setConstant(0xffffff);
                        break;
                }
            }

            if(Controls.Climbing){
                ActiveIntake.powerOff();
                DropDown.GoUp();
                isClimbing = true;
                Climb.engagePTO();
                Climb.Raise();
                Controls.Climbing = false;
            }
            if(isClimbing){
                if(Controls.gamepad2.wasPressed.touchpad) {
                    Climb.PutDown();
                }
                Climb.Update();
                Controls.Update();
                continue;
            }
            Chassis.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.left_trigger - gamepad1.right_trigger);

            Controls.Update();
            OutTakeController.Update();
            Elevator.update();
            Arm.update();
            DropDown.Update();
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

