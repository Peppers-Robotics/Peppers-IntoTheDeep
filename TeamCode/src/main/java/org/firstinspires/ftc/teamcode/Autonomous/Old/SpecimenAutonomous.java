package org.firstinspires.ftc.teamcode.Autonomous.Old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SpecimenAutonomous extends LinearOpMode {
    public enum States{
        SCORE,
        GO_TO_SAMPLES,
        GO_TO_HUMAN,
        TAKE_SPECIMEN,
        PARK
    }


    public static double[] specimenX = {-39, -40, -40, -40, -40}, specimenY = {-16, -15, -14, -13, -12}, specimenH = {0, 0, 0, 0, 0};
    public static double[] sampleX = {-19, -22, -26}, sampleY = {16, 21.5, 31}, sampleH = {313, 307, 300};
    public static double[] getSpecimenX = {3, 2, 1, 1}, getSpecimenY = {18.7, 18, 17, 17}, getSpecimenH = {0, 355, 0, 0};
    public static double[] ReverseToHumanX = {-18, -21, -26}, ReverseToHumanY = {15, 30, 31.5}, ReverseToHumanH = {215, 213, 200};
    public static double parkX = 0, parkY = 22, parkH = 0;
    public static int[] ExtendoPose = {720, 710, 620}, ExtendoPoseHuman = {600, 400, 400};
    SampleMecanumDriveCancelable drive;
    public int specimensScored = 0, samplesTook = 0;
    public static double dropDownPose = 0.65;

    private TrajectorySequence ScoreSpecimen(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI*2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(specimenX[specimensScored], specimenY[specimensScored], Math.toRadians(specimenH[specimensScored])))
                .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                    driveIsFree = true;
                })
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToSample(){
        if(specimensScored == 1 && samplesTook == 0){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToSplineHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])), Math.toRadians(100))
                    .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> Extendo.Extend(ExtendoPose[samplesTook]))
                    .addTemporalMarker(() -> samplesTook ++)
                    .build();
        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> Extendo.Extend(ExtendoPose[samplesTook]))
                .addTemporalMarker(() -> samplesTook ++)
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToHuman(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(ReverseToHumanX[samplesTook - 1], ReverseToHumanY[samplesTook - 1], Math.toRadians(ReverseToHumanH[samplesTook - 1])))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Extendo.Extend(ExtendoPoseHuman[samplesTook - 1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                    driveIsFree = true;
                })
                .resetConstraints()
                .build();
    }
    private TrajectorySequence TakeFromWall(){
        OutTakeStateMachine.ElevatorTakeSpecimen -= 10;
        if(specimensScored == 1){
            return drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(getSpecimenH[specimensScored - 1])))
                    .addTemporalMarker(() -> {
                        Elevator.controller.pidCoefficients.i = 0.05;
                        OutTakeStateMachine.autoTakingSamples = false;
//                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
//                        OutTakeStateMachine.ElevatorTakeSpecimen += 10;
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                    })
//                    .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI * 2, DriveConstants.TRACK_WIDTH))
//                    .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
//                    .splineToSplineHeading(new Pose2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1], Math.toRadians(getSpecimenH[specimensScored - 1])), Math.toRadians(-170))
                    .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1] - 5, getSpecimenY[specimensScored - 1], Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1], Math.toRadians(0)))
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE) {
                            OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
//                            OutTakeStateMachine.ElevatorTakeSpecimen -= 10;
                            Elevator.controller.pidCoefficients.i = 0;
                            OutTakeStateMachine.ElevatorTakeSpecimen += 10;
                        }
                    })
                    .resetConstraints()
                    .build();

        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    Elevator.controller.pidCoefficients.i = 0.05;
                    OutTakeStateMachine.autoTakingSamples = false;
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .addTemporalMarker(() -> {
                    SampleMecanumDriveCancelable.LATERAL_MULTIPLIER = 1.7;
                })
                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), Math.toRadians(-60))
                .addTemporalMarker(() -> {
                    SampleMecanumDriveCancelable.LATERAL_MULTIPLIER = 1.5;
                })

//                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1] - 20, getSpecimenY[specimensScored - 1]), Math.toRadians(30))
//                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(30))
//                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(40, Math.PI * 2, DriveConstants.TRACK_WIDTH))
//                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE)
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        OutTakeStateMachine.ElevatorTakeSpecimen += 8;
                })
                .build();
    }
    private static ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator.RESET = true;
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap);
        Claw.close();
        Controls.Initialize(gamepad1, gamepad2);
        OutTakeController.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        Initialization.telemetry = new MultipleTelemetry(Initialization.telemetry, telemetry);
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
                        specimensScored ++;
                        if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE)
                            OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        trajRan = false;
                        if(specimensScored >= 4) {
                            CurrentState = States.PARK;
                            break;
                        }
                        if(samplesTook < 3) CurrentState = States.GO_TO_SAMPLES;
                        else {
                            CurrentState = States.TAKE_SPECIMEN;
                            Extendo.pidEnable = false;
                        }
                    }
                    break;
                case GO_TO_SAMPLES:
                    if(!drive.isBusy() && !trajRan){
                        trajRan = true;
                        drive.followTrajectorySequenceAsync(GoToSample());
                        OutTakeStateMachine.autoTakingSamples = true;
                        time.reset();
                        break;
                    }
                    if(drive.isBusy()) time.reset();
                    if(trajRan && ((time.seconds() >= 0.15 && !drive.isBusy()) || (!Storage.isStorageEmpty()))){
                        trajRan = false;
                        Extendo.Extend(100);
                        CurrentState = States.GO_TO_HUMAN;
                    } else {
                        IntakeController.gamepad2.right_trigger = (float) dropDownPose;
                        OutTakeStateMachine.autoTakingSamples = false;
//                        DropDown.setInstantPosition(dropDownPose);
                    }
                    break;
                case GO_TO_HUMAN:
                    if(!drive.isBusy() && !trajRan){
                        drive.followTrajectorySequenceAsync(GoToHuman());
                        Extendo.Extend(100);
                        ActiveIntake.power = 0.7;
                        trajRan = true;
                        driveIsFree = false;
                    }
                    if(!driveIsFree) timeHuman.reset();
                    if(driveIsFree && trajRan && timeHuman.seconds() >= 0.1){
                        drive.breakFollowing();
                        IntakeController.gamepad2.gamepad.left_bumper = true;
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
//                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
//                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    }
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    if(Arm.getCurrentArmAngle() < 300 && Claw.isClosed() && Elevator.getTargetPosition() == OutTakeStateMachine.ElevatorScoreSpecimen){
                        drive.breakFollowing();
                    }

                    if(!drive.isBusy() && trajRan){
                        CurrentState = States.SCORE;
                        trajRan = false;
                    }
                    break;
                case PARK:
                    if(!trajRan) {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(70))
                                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkH))).build());
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
        OutTakeStateMachine.autoTakingSamples = false;
    }
    public static ElapsedTime freq = new ElapsedTime(), timeHuman = new ElapsedTime();
    public static boolean driveIsFree = false;
}