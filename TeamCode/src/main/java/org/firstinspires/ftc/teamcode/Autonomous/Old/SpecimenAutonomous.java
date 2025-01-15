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

@Autonomous(name = "5+0 (specimen)")
@Config
public class SpecimenAutonomous extends LinearOpMode {
    public enum States{
        SCORE,
        GO_TO_SAMPLES,
        GO_TO_HUMAN,
        TAKE_SPECIMEN,
        PARK
    }


    public static double[] specimenX = {-38, -38, -38, -38, -38}, specimenY = {-10, -11.5, -13.5, -16, -18}, specimenH = {0, 0, 0, 0, 0};
    public static double[] sampleX = {-19, -19, -19}, sampleY = {20, 20, 34}, sampleH = {325, 333, 315};
    public static double[] getSpecimenX = {4, 1, 1, 1}, getSpecimenY = {17, 18, 18, 18}, getSpecimenH = {0, 0, 0, 0};
    public static double[] ReverseToHumanX = {-19, -19, -19}, ReverseToHumanY = {19.1, 18, 26.1}, ReverseToHumanH = {220, 210, 210};
    public static double parkX = 0, parkY = 22, parkH = 0;
    public static int[] ExtendoPose = {690, 760, 720}, ExtendoPoseHuman = {600, 500, 200};
    SampleMecanumDriveCancelable drive;
    public int specimensScored = 0, samplesTook = 0;
    public static double dropDownPose = 0.7;

    private TrajectorySequence ScoreSpecimen(){
        if(specimensScored == 0){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI*2, DriveConstants.TRACK_WIDTH))
                    .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))

                    .lineToLinearHeading(new Pose2d(specimenX[specimensScored], specimenY[specimensScored]))

                    .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                        driveIsFree = true;
                    })
                    .setReversed(false)
                    .resetConstraints()
                    .build();
        }
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, Math.PI*2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))

                .splineToLinearHeading(new Pose2d(specimenX[specimensScored] + 10, specimenY[specimensScored], Math.toRadians(specimenH[specimensScored])), Math.toRadians(180))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(20, Math.PI*2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))

                .splineToConstantHeading(new Vector2d(specimenX[specimensScored], specimenY[specimensScored]), Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    driveIsFree = true;
                })
                .setReversed(false)
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToSample(){
        if(specimensScored == 1 && samplesTook == 0){
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])), Math.toRadians(120))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.RETRACT);
                        Extendo.Extend(ExtendoPose[samplesTook]);
                        DropDown.setInstantPosition(dropDownPose);
                    })
                    .addTemporalMarker(() -> samplesTook ++)
                    .build();
        }
        double tw = DriveConstants.TRACK_WIDTH;
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    Extendo.Extend(20);
                    DropDown.setInstantPosition(0);
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 2, tw))
                .lineToLinearHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, () -> {
                    Extendo.Extend(ExtendoPose[samplesTook]);
                    DropDown.setInstantPosition(dropDownPose);
                })
                .addTemporalMarker(() -> samplesTook ++)
                .resetConstraints()
                .build();
    }
    private TrajectorySequence GoToHuman(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    Extendo.Extend(100);
                    DropDown.setInstantPosition(0);
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(DriveConstants.MAX_VEL, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(ReverseToHumanX[samplesTook - 1], ReverseToHumanY[samplesTook - 1], Math.toRadians(ReverseToHumanH[samplesTook - 1])))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    Extendo.Extend(ExtendoPoseHuman[samplesTook - 1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                    driveIsFree = true;
                })
                .resetConstraints()
                .build();
    }
    private TrajectorySequence TakeFromWall(){
        if(specimensScored == 1){
            return drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(getSpecimenH[specimensScored - 1])))
                    .resetConstraints()
                    .addTemporalMarker(() -> {
                        OutTakeStateMachine.autoTakingSamples = false;
                        Extendo.Extend(0);
                        Extendo.pidEnable = true;
//                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                    })
//                    .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, Math.PI * 2, DriveConstants.TRACK_WIDTH))
//                    .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
//                    .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1] - 5, getSpecimenY[specimensScored - 1], Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(getSpecimenX[specimensScored - 1] - 5, getSpecimenY[specimensScored - 1]))
                    .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(40, Math.PI*2, DriveConstants.TRACK_WIDTH))
                    .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(30))
//                    .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1], Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), Math.toRadians(0))
                    .waitSeconds(0.05)
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    })
                    .build();

        }
        return drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(getSpecimenH[specimensScored - 1])))
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.autoTakingSamples = false;
//                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, Math.PI * 2, DriveConstants.TRACK_WIDTH))

//                .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1] - 5, getSpecimenY[specimensScored - 1], Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1] - 10, getSpecimenY[specimensScored - 1]), Math.toRadians(0))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(40, Math.PI*2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(30))

                .splineToConstantHeading(new Vector2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1]), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                })
                .resetConstraints()
                .build();
    }
    private static ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator.RESET = true;
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap, false);
        Claw.close();
        Controls.Initialize(gamepad1, gamepad2);
        OutTakeController.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        Initialization.telemetry = new MultipleTelemetry(Initialization.telemetry, telemetry);
        Initialization.Team = Initialization.AllianceColor.RED;

        OutTakeStateMachine.ElevatorScoreSpecimen = 0;
        double tmp = OutTakeStateMachine.ArmScoreSpecimen;
        OutTakeStateMachine.ArmScoreSpecimen = 90;
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
            OutTakeStateMachine.ArmScoreSpecimen = tmp;
            Initialization.updateCacheing();
            switch (CurrentState){
                case SCORE:
                    if(!drive.isBusy() && !trajRan) {
                        drive.followTrajectorySequenceAsync(ScoreSpecimen());
                        trajRan = true;
                        driveIsFree = false;
                    }
                    if(drive.getPoseEstimate().getX() < -33 && trajRan){
                        drive.breakFollowing();
                        specimensScored ++;
                        if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE) {
                            if(specimensScored == 1 || specimensScored == 5) OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                            else OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                        }
                        trajRan = false;
                        if(specimensScored >= 5) {
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
                        DropDown.setInstantPosition(0);
                        time.reset();
                        break;
                    }
                    if(drive.isBusy()) time.reset();
                    if(trajRan && ((time.seconds() >= 0.3 && !drive.isBusy()) || (!Storage.isStorageEmpty()))){
//                        IntakeController.gamepad2.right_trigger = (float) 0.11;
                        DropDown.setInstantPosition(0);
                        trajRan = false;
                        ActiveIntake.powerOn((float)0.4);
                        CurrentState = States.GO_TO_HUMAN;
                    } else {
//                        IntakeController.gamepad2.right_trigger = (float) dropDownPose;
                        ActiveIntake.powerOn();
                        OutTakeStateMachine.autoTakingSamples = false;
                        DropDown.setInstantPosition(dropDownPose);
                    }
                    break;
                case GO_TO_HUMAN:
                    if(!drive.isBusy() && !trajRan){
                        drive.followTrajectorySequenceAsync(GoToHuman());
                        ActiveIntake.power = 0.85;
                        trajRan = true;
                        driveIsFree = false;
                    }
                    if(!driveIsFree) timeHuman.reset();
                    if(driveIsFree && trajRan && timeHuman.seconds() >= 0.1){
                        drive.breakFollowing();
//                        IntakeController.gamepad2.right_trigger = 0;
//                        IntakeController.gamepad2.gamepad.left_bumper = true;
//                        IntakeController.Update(false);
                        ActiveIntake.Reverse();
                        DropDown.setInstantPosition(0);
                        if(time.seconds() >= 0.3){
                            trajRan = false;
                            if(samplesTook < 3) CurrentState = States.GO_TO_SAMPLES;
                            else {
                                CurrentState = States.TAKE_SPECIMEN;
//                                IntakeController.gamepad1.right_stick_y = 1;
                                Extendo.Extend(0);
                            }
//                            IntakeController.gamepad2.gamepad.left_bumper = false;
                            ActiveIntake.powerOn();
                        }
                    } else time.reset();
                    break;
                case TAKE_SPECIMEN:
                    if(!drive.isBusy() && !trajRan){
                        Extendo.pidEnable = true;
                        Extendo.Extend(0);
//                        IntakeController.gamepad2.left_trigger = 0;
//                        IntakeController.gamepad2.right_trigger = 0;
                        ActiveIntake.powerOff();
                        DropDown.setInstantPosition(0);
                        trajRan = true;
                        drive.followTrajectorySequenceAsync(TakeFromWall());
//                        OutTakeStateMachine.ChangeStateTo(OutTakeStateMachine.OutTakeStates.ELEVATOR_TO_SPECIMEN_TAKE);
//                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    }
                    if(OutTakeStateMachine.CurrentState == OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_TAKE) Elevator.PowerOnDownToTakeSample = true;
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                    if(Arm.getCurrentArmAngle() < 300 && Claw.isClosed()){
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
                                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(90, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(80))
                                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkH))).build());
                        trajRan = true;
                    }
                    break;
            }
            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            DropDown.Update();
//            IntakeController.Update(true);
            OutTakeController.Update();
            Initialization.telemetry.addData("htz", 1/freq.seconds());
            Initialization.telemetry.update();
            freq.reset();
        }
        OutTakeStateMachine.autoTakingSamples = false;
        OutTakeStateMachine.ArmScoreSpecimen = tmp;
    }
    public static ElapsedTime freq = new ElapsedTime(), timeHuman = new ElapsedTime();
    public static boolean driveIsFree = false;
}