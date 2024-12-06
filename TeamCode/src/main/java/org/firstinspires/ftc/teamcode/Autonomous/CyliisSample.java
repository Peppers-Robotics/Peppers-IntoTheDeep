package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.opmode.StrafeTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous
public class CyliisSample extends LinearOpMode {

    public enum States{
        PLACE_SAMPLE,
        TAKE_SAMPLE_HUMAN,
        GOTO_SAMPLE1,
        GOTO_SAMPLE2,
        GOTO_SAMPLE3,
        TAKE_SAMPLE,
        GOTO_BASKET_AND_SCORE,
        SCORE,
        IDLE, GOTO_BASKETH_AND_SCORE;
        public boolean trajRan = false;
    }
    public static States CurrentState = States.PLACE_SAMPLE;
    public static int takeSample1Extend = 700, takeSample2Extend = 720, takeSample3Extend = 720;
    public static double SlowExtendoPower = -0.4, dropDownPos = 0.6;

    public static Pose2d putSample = new Pose2d(-10, -46.5, Math.toRadians(318)),
            takeSamplehuman = new Pose2d(-2, 31, Math.toRadians(270)),
            takeSample1 = new Pose2d(-11, -41.5, Math.toRadians(350)),
            takeSample2 = new Pose2d(-11, -40.5, Math.toRadians(16)),
            takeSample3 = new Pose2d(-12, -37, Math.toRadians(40)),
            basketPosition = new Pose2d(-4.5, -43 ,Math.toRadians(330)),
            basketHumanPosition = new Pose2d(-10, -45, Math.toRadians(318));
    public static int samplesScored = 0;
    public static ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        OutTakeController.Initialize(gamepad1, gamepad2);
        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SAMPLE_SCORE);
        OutTakeStateMachine.ElevatorScoreSpecimen = 0;
        samplesScored = 0;
        CurrentState = States.PLACE_SAMPLE;
        DropDown.GoUp();

        OutTakeStateMachine.inAuto = true;
        IntakeController.autoIntake = true;
        IntakeController.isInAuto = true;

        Initialization.Team = Initialization.AllianceColor.RED;

        TrajectorySequence putSampleT = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.ElevatorScoreSample = OutTakeStateMachine.ElevatorSample2;
                })
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(putSample)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    OutTakeStateMachine.inAuto = false;
                })

                .build();

        TrajectorySequence takeSampleHuman = drive.trajectorySequenceBuilder(putSampleT.end())
                .lineToLinearHeading(takeSamplehuman)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    IntakeController.gamepad2.right_trigger = (float) dropDownPos;
                    Extendo.Extend(takeSample2Extend);
                })
                .build();

        while (opModeInInit()){
            Claw.close();
            Initialization.updateCacheing();
            Elevator.update();
            Arm.update();
            IntakeController.Update();
            OutTakeController.Update();
        }
        CurrentState = States.PLACE_SAMPLE;
        CurrentState.trajRan = true;
        drive.followTrajectorySequenceAsync(putSampleT);

        while (opModeIsActive()){
            Initialization.updateCacheing();

            switch (CurrentState) {
                case PLACE_SAMPLE:
                    if (!drive.isBusy()) {
                        if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.RETRACT_ARM){
                            drive.followTrajectorySequenceAsync(takeSampleHuman);
                            CurrentState = States.TAKE_SAMPLE_HUMAN;
                            CurrentState.trajRan = false;
                        }
                    }
                    break;
                case TAKE_SAMPLE_HUMAN:
                    if(Storage.hasAlliancePice()){
                        Extendo.pidEnable = false;
                        IntakeController.gamepad1.right_stick_y = 1;
                        CurrentState = States.GOTO_BASKET_AND_SCORE;
                        CurrentState.trajRan = false;
                        if(drive.isBusy())
                            drive.breakFollowing();
                        break;
                    }
                    break;
                case GOTO_SAMPLE1:
                    if (!CurrentState.trajRan) {
                        CurrentState.trajRan = true;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(takeSample1)
                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                                    Extendo.Extend(takeSample2Extend);

                                })
                                .build());
                        IntakeController.gamepad2.right_trigger = (float) dropDownPos;
                    }
                    if (!drive.isBusy()) {
//                        Extendo.Extend(takeSample1Extend, 20);
                        CurrentState = States.TAKE_SAMPLE;
                        CurrentState.trajRan = false;
                    }
                    break;
                case GOTO_SAMPLE2:
                    if (!CurrentState.trajRan) {
                        CurrentState.trajRan = true;
                        IntakeController.gamepad2.right_trigger = (float) dropDownPos;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(takeSample2)
                                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                                    Extendo.Extend(takeSample2Extend);

                                })
                                .build());
                    }
                    if (!drive.isBusy()) {
                        CurrentState = States.TAKE_SAMPLE;
                        CurrentState.trajRan = false;
                    }
                    break;
                case GOTO_SAMPLE3:
                    if (!CurrentState.trajRan) {
                        CurrentState.trajRan = true;
                        IntakeController.gamepad2.right_trigger = (float) dropDownPos;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(takeSample3)
                                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                                    Extendo.Extend(takeSample3Extend);
                                })
                                .build());
                    }
                    if (!drive.isBusy()) {
                        CurrentState = States.TAKE_SAMPLE;
                        CurrentState.trajRan = false;
                    }
                    break;
                case TAKE_SAMPLE:
                    if (!CurrentState.trajRan) {
//                        Extendo.motor.setPower(SlowExtendoPower);
                        CurrentState.trajRan = true;
                        IntakeController.gamepad2.right_trigger = (float) dropDownPos;
                    }
                    if (Storage.hasAlliancePice()) {
                        Extendo.pidEnable = false;
                        IntakeController.gamepad1.right_stick_y = 1;
                        CurrentState = States.GOTO_BASKET_AND_SCORE;
                        CurrentState.trajRan = false;
                    }
                    break;
                case GOTO_BASKET_AND_SCORE:
                    if (!CurrentState.trajRan) {
                        if(samplesScored == 0){
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(basketHumanPosition)
                                    .build()
                            );
                        } else {
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(basketPosition)
                                    .build()
                            );
                        }
                        CurrentState.trajRan = true;
                    }
                    if (IntakeController.CurrentState == IntakeController.IntakeStates.RETRACT_EXTENDO) {
                        IntakeController.gamepad1.right_stick_y = 0;
                        IntakeController.gamepad2.right_trigger = 0;
//                        IntakeController.gamepad1.update();
//                        IntakeController.gamepad2.update();
                    }
                    if (!drive.isBusy() && IntakeController.CurrentState == IntakeController.IntakeStates.IDLE_RETRACTED) {
                        CurrentState = States.SCORE;
                    }
                    break;
                case SCORE:
                    if (OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WITH_SAMPLE) {
                        OutTakeStateMachine.ElevatorScoreSample = OutTakeStateMachine.ElevatorSample2;
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SAMPLE);
                    }
                    if(!drive.isBusy() && OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SAMPLE_SCORE){
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    }
                    if (OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.RETRACT_ELEVATOR) {
                        switch (samplesScored) {
                            case 0:
                                samplesScored ++;
                                CurrentState = States.GOTO_SAMPLE1;
                                CurrentState.trajRan = false;
                                break;
                            case 1:
                                samplesScored ++;
                                CurrentState = States.GOTO_SAMPLE2;
                                CurrentState.trajRan = false;
                                break;
                            case 2:
                                samplesScored ++;
                                CurrentState = States.GOTO_SAMPLE3;
                                CurrentState.trajRan = false;
                                break;
                            case 3:
                                samplesScored ++;
                                CurrentState = States.IDLE;
                                CurrentState.trajRan = false;
                                break;

                        }
                    }
                    break;
            }

            drive.update();
            Elevator.update();
            Extendo.update();
            Arm.update();
            IntakeController.Update(true);
            OutTakeController.Update();
            if(IntakeController.gamepad2.right_trigger <= 0.01) DropDown.setInstantPosition(0);
            DropDown.Update();
            Initialization.telemetry.addData("AUTO state", CurrentState.toString());
            Initialization.telemetry.addData("Extendo to targetpos", Extendo.ReachedTargetPosition(10));
            Initialization.telemetry.addData("IntakeController", IntakeController.CurrentState.toString());
            Initialization.telemetry.update();
        }
        IntakeController.autoIntake = false;
    }
}