package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.Controls;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class SpecimenAutonomous extends LinearOpMode {
    public static double[] specimenX = {0, 0, 0, 0, 0}, specimenY = {0, 0, 0, 0, 0}, specimenH = {0, 0, 0, 0};
    public static double[] sampleX = {0, 0, 0}, sampleY = {0, 0, 0}, sampleH = {0, 0, 0};
    public static double[] getSpecimenX = {0, 0, 0, 0}, getSpecimenY = {0, 0, 0, 0}, getSampleH = {0, 0, 0, 0};
    public static double[] ReverseToHumanX = {0, 0, 0}, ReverseToHumanY = {0, 0, 0}, ReverseToHumanH = {0, 0, 0};


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        Initialization.initializeRobot(hardwareMap);

        OutTakeStateMachine.CurrentState = OutTakeStateMachine.OutTakeStates.IDLE_WHILE_SPECIMEN_SCORE;
        OutTakeStateMachine.ElevatorScoreSpecimen = 0;

//        TrajectorySequence ScoreFromInit = new TrajectorySequence()


        while(opModeInInit()){

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update();
            OutTakeController.Update();
        }

        while(isStarted()){

            drive.update();
            Elevator.update();
            Arm.update();
            Extendo.update();
            IntakeController.Update();
            OutTakeController.Update();
        }
    }
}
