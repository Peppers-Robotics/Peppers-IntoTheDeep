package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SpecimenAutonomous extends LinearOpMode {
    public enum States{
        SCORE,
        GO_TO_SAMPLES,
        TAKE_SAMPLES,
        GO_TO_HUMAN,
        SPIT,
        TAKE_SPECIMEN,
        SCORE_SPECIMEN,
        PARK
    }


    public static double[] specimenX = {-39, -40, -41, -42, -43}, specimenY = {-12, -14, -16, -18, -20}, specimenH = {0, 0, 0, 0, 0};
    public static double[] sampleX = {-19, -21, -26}, sampleY = {16, 21, 31}, sampleH = {310, 315, 310};
    public static double[] getSpecimenX = {5, 2, 2, 2}, getSpecimenY = {19, 18.5, 18.5, 19}, getSpecimenH = {5, 3, 2, 3};
    public static double[] ReverseToHumanX = {-10, -21, -22}, ReverseToHumanY = {12, 30, 35}, ReverseToHumanH = {215, 213, 190};
    public static double parkX = 0, parkY = 18, parkH = 0;
    public static int[] ExtendoPose = {700, 690, 600}, ExtendoPoseHuman = {500, 400, 450};
    SampleMecanumDriveCancelable drive;
    public int specimensScored = 0, samplesTook = 0;
    public static double dropDownPose = 0.65;

    private TrajectorySequence ScoreSpecimen(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(80, Math.PI*2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(specimenX[specimensScored], specimenY[specimensScored], Math.toRadians(specimenH[specimensScored])))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    driveIsFree = true;
                })
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToSample(){
        if(specimensScored == 1 && samplesTook == 0){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(4)
                    .addTemporalMarker(ActiveIntake::BlockIntake)
                    .splineToSplineHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])), Math.toRadians(70))
                    .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> Extendo.Extend(ExtendoPose[samplesTook]))
                    .addTemporalMarker(() -> samplesTook ++)
                    .build();
        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(ActiveIntake::BlockIntake)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 3, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Extendo.Extend(ExtendoPose[samplesTook]))
                .addTemporalMarker(() -> samplesTook ++)
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToHuman(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 2.5, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(ReverseToHumanX[samplesTook - 1], ReverseToHumanY[samplesTook - 1], Math.toRadians(ReverseToHumanH[samplesTook - 1])))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Extendo.Extend(ExtendoPoseHuman[samplesTook - 1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    driveIsFree = true;
                })
                .resetConstraints()
                .build();
    }
    private TrajectorySequence TakeFromWall(){
        if(specimensScored == 1){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(() -> {
                        Elevator.PowerOnDownToTakeSample = true;
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    })
                    .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
                    .splineToSplineHeading(new Pose2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1], Math.toRadians(getSpecimenH[specimensScored - 1])), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        Elevator.PowerOnDownToTakeSample = true;
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    })
                    .resetConstraints()
                    .build();

        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(65))
                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1] - 4, getSpecimenY[specimensScored - 1]), Math.toRadians(30))
//                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                })
                .build();
    }
    private static ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap);
        Controls.Initialize(gamepad1, gamepad2);
        OutTakeController.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        Initialization.Team = Initialization.AllianceColor.RED;

        OutTakeStateMachine.ElevatorScoreSpecimen = 0;
        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE);
        States CurrentState = States.SCORE;
        boolean trajRan = false;
        Extendo.pidEnable = false;


        Claw.close();
        while(opModeInInit()){
            Initialization.updateCacheing();

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update();
            OutTakeController.Update();
        }

        OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
        ActiveIntake.BlockIntake();

        while(opModeIsActive()){
            Initialization.updateCacheing();
            switch (CurrentState){
                case SCORE:
                    if(!drive.isBusy() && !trajRan) {
                        drive.followTrajectorySequenceAsync(ScoreSpecimen());
                        trajRan = true;
                        driveIsFree = false;
                    }
                    if(driveIsFree && trajRan){
                        drive.breakFollowing();
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        specimensScored ++;
                        trajRan = false;
                        if(specimensScored >= 5) {
                            CurrentState = States.PARK;
                            break;
                        }
                        if(samplesTook < 3) CurrentState = States.GO_TO_SAMPLES;
                        else {
                            OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                            CurrentState = States.TAKE_SPECIMEN;
                            Extendo.pidEnable = false;
                        }
                    }
                    break;
                case GO_TO_SAMPLES:
                    if(!drive.isBusy() && !trajRan){
                        trajRan = true;
                        drive.followTrajectorySequenceAsync(GoToSample());
                        time.reset();
                        break;
                    }
                    if(drive.isBusy()) time.reset();
                    if(!drive.isBusy() && trajRan && time.seconds() >= 0.15){
                        trajRan = false;
                        CurrentState = States.GO_TO_HUMAN;
                    } else {
                        ActiveIntake.power = 1;
                        IntakeController.gamepad2.right_trigger = (float) dropDownPose;
//                        DropDown.setInstantPosition(dropDownPose);
                    }
                    break;
                case GO_TO_HUMAN:
                    if(!drive.isBusy() && !trajRan){
                        drive.followTrajectorySequenceAsync(GoToHuman());
                        ActiveIntake.power = 0.3;
                        trajRan = true;
                        driveIsFree = false;
                    }
                    if(driveIsFree && trajRan){
                        drive.breakFollowing();
                        IntakeController.gamepad2.gamepad.left_bumper = true;
                        ActiveIntake.power = 1;
                        IntakeController.Update(false);
                        ActiveIntake.UnblockIntake();
                        if(time.seconds() >= 0.3){
                            trajRan = false;
                            Extendo.Extend(100);
                            if(samplesTook < 3) CurrentState = States.GO_TO_SAMPLES;
                            else {
                                CurrentState = States.TAKE_SPECIMEN;
                                IntakeController.gamepad1.right_stick_y = 1;
                                Extendo.pidEnable = false;
                            }
                            IntakeController.gamepad2.gamepad.left_bumper = false;
                        }
                    } else time.reset();
                    break;
                case TAKE_SPECIMEN:
                    if(!drive.isBusy() && !trajRan){
                        Extendo.pidEnable = false;
                        trajRan = true;
                        drive.followTrajectorySequenceAsync(TakeFromWall());
                    }
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    if(!drive.isBusy() && trajRan){
                        CurrentState = States.SCORE;
                        trajRan = false;
                    }
                    break;
                case PARK:
                    if(!trajRan) {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkH))).build());
                        trajRan = true;
                    }
                    break;
            }
            if(IntakeController.CurrentState == IntakeController.IntakeStates.RETRACT_EXTENDO && samplesTook >= 3)
                gamepad1.right_stick_y = 0;

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            DropDown.Update();
            IntakeController.Update(true);
            OutTakeController.Update();
            Initialization.telemetry.addData("htz", 1/freq.seconds());
            Initialization.telemetry.update();
            freq.reset();
        }
    }
    public static ElapsedTime freq = new ElapsedTime();
    public static boolean driveIsFree = false;
}