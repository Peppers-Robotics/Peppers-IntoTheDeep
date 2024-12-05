package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class RedSample extends LinearOpMode {

    public enum States{
        PLACE_SPECIMEN,
        GOTO_SAMPLE1,
        TAKE_SAMPLE,
        GOTO_BASKET_AND_SCORE,
    }
    public static States CurrentState = States.PLACE_SPECIMEN;

    public static Pose2d putSpecimen = new Pose2d(-34.5, 1, 0),
                         takeSample1 = new Pose2d(0, 0, 0),
                         takeSample2 = new Pose2d(0, 0, 0),
                         takeSample3 = new Pose2d(0, 0, 0),
                         basketPosition = new Pose2d(0, 0 ,0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        OutTakeController.Initialize(gamepad1, gamepad2);
        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE);
        OutTakeStateMachine.ElevatorScoreSpecimen = 0;

        TrajectorySequence putSpecimenT = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen1;
                })
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .waitSeconds(0.5)
                .lineToLinearHeading(putSpecimen)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                })
                .waitSeconds(0.1)

                .build();

        while (opModeInInit()){
            Initialization.updateCacheing();
            Elevator.update();
            Arm.update();
            IntakeController.Update();
            OutTakeController.Update();
        }
        drive.followTrajectorySequenceAsync(putSpecimenT);

        while (opModeIsActive()){
            Initialization.updateCacheing();
            drive.update();

            switch (CurrentState){
                case PLACE_SPECIMEN:

                    break;
            }

            Elevator.update();
            Arm.update();
            IntakeController.Update();
            OutTakeController.Update();
            Controls.Update();
        }
    }
}
