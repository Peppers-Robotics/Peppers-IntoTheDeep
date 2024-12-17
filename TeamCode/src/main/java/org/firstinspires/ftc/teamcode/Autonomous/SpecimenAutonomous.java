package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import javax.crypto.Cipher;

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


    public static double[] specimenX = {0, 0, 0, 0, 0}, specimenY = {0, 0, 0, 0, 0}, specimenH = {0, 0, 0, 0};
    public static double[] sampleX = {0, 0, 0}, sampleY = {0, 0, 0}, sampleH = {0, 0, 0};
    public static double[] getSpecimenX = {0, 0, 0, 0}, getSpecimenY = {0, 0, 0, 0}, getSampleH = {0, 0, 0, 0};
    public static double[] ReverseToHumanX = {0, 0, 0}, ReverseToHumanY = {0, 0, 0}, ReverseToHumanH = {0, 0, 0};
    public static double parkX = 0, parkY = 0, parkH = 0;
    public static int[] ExtendoPose = {0, 0, 0};
    SampleMecanumDriveCancelable drive;
    public int specimensScored = 0, samplesTook = 0;
    public static double dropDownPose = 0.7;

    private TrajectorySequence ScoreSpecimen(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(specimenX[specimensScored], specimenY[specimensScored], Math.toRadians(specimenH[specimensScored])))
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                    specimensScored ++;
                })
                .build();
    }
    private TrajectorySequence GoToSample(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(ActiveIntake::BlockIntake)
                .lineToLinearHeading(new Pose2d(sampleX[samplesTook], sampleY[samplesTook], Math.toRadians(sampleH[samplesTook])))
                .addTemporalMarker(() -> {
                    Extendo.Extend(ExtendoPose[samplesTook]);
                    samplesTook ++;
                })
                .build();
    }
    private TrajectorySequence GoToHuman(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(ReverseToHumanX[samplesTook - 1], ReverseToHumanY[samplesTook - 1], Math.toRadians(ReverseToHumanH[samplesTook - 1])))
                .build();
    }
    private TrajectorySequence TakeFromWall(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SPECIMEN);
                })
                .lineToLinearHeading(new Pose2d(getSpecimenX[specimensScored - 1], getSpecimenY[specimensScored - 1], Math.toRadians(getSampleH[specimensScored - 1])))
                .waitSeconds(0.2)
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

        OutTakeStateMachine.ElevatorScoreSpecimen = 0;
        OutTakeStateMachine.CurrentState = OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE;
        States CurrentState = States.SCORE;
        boolean trajRan = false;


        while(opModeInInit()){

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update();
            OutTakeController.Update();
        }

        OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;

        while(isStarted()){
            switch (CurrentState){
                case SCORE:
                    if(!drive.isBusy() && !trajRan) {
                        drive.followTrajectorySequenceAsync(ScoreSpecimen());
                        trajRan = true;
                    }
                    if(!drive.isBusy() && trajRan){
                        trajRan = false;
                        if(samplesTook >= 5) CurrentState = States.PARK;
                        if(samplesTook < 3) CurrentState = States.GO_TO_SAMPLES;
                        else CurrentState = States.TAKE_SPECIMEN;
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
                    if(!drive.isBusy() && trajRan && time.seconds() >= 0.3){
                        ActiveIntake.Reverse();
                        if(time.seconds() >= 0.3 + 0.2){
                            trajRan = false;
                            CurrentState = States.GO_TO_HUMAN;
                        }
                    } else {
                        ActiveIntake.powerOn();
                        DropDown.setInstantPosition(dropDownPose);
                    }
                    break;
                case GO_TO_HUMAN:
                    if(!drive.isBusy() && !trajRan){
                        drive.followTrajectorySequenceAsync(GoToHuman());
                        trajRan = true;
                    }
                    if(!drive.isBusy() && trajRan){
                        ActiveIntake.Reverse();
                        if(time.seconds() >= 0.2){
                            trajRan = false;
                            if(samplesTook < 3) CurrentState = States.TAKE_SAMPLES;
                            else CurrentState = States.TAKE_SPECIMEN;
                            ActiveIntake.powerOff();
                        }
                    } else time.reset();
                    break;
                case TAKE_SPECIMEN:
                    if(!drive.isBusy() && !trajRan){
                        IntakeController.gamepad1.right_stick_y = 1;
                        trajRan = true;
                        drive.followTrajectorySequenceAsync(TakeFromWall());
                    }
                    if(!drive.isBusy() && trajRan){
                        CurrentState = States.SCORE;
                        trajRan = false;
                    }
                    break;
                case PARK:
                    if(!trajRan) {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkH))).build());
                    }
            }
            if(IntakeController.CurrentState == IntakeController.IntakeStates.RETRACT_EXTENDO)
                gamepad1.right_stick_y = 0;

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update(true);
            OutTakeController.Update();
        }
    }
}