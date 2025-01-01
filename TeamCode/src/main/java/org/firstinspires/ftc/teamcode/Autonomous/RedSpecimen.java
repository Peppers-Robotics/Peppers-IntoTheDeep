package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.RobotRelevantClasses.TunablePose2d;
import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.IntakeController;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeController;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeStateMachine;
import org.firstinspires.ftc.teamcode.Tasks.GoToPoint;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Autonomous
@Config
public class RedSpecimen extends LinearOpMode {
    public static SparkFunOTOS.Pose2D scoreSpecimen = new SparkFunOTOS.Pose2D(0, 0, 0),
    presample1 = new SparkFunOTOS.Pose2D(0, 0, 0),
    sample1 = new SparkFunOTOS.Pose2D(0, 0, 0),
    sample2 = new SparkFunOTOS.Pose2D(0, 0, 0),
    sample3 = new SparkFunOTOS.Pose2D(0, 0, 0),
    pretakeSpecimen = new SparkFunOTOS.Pose2D(0, 0 ,0),
    takeSpecimen = new SparkFunOTOS.Pose2D(0, 0, 0);

    public static Scheduler scheduler;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization.initializeRobot(hardwareMap);
        Initialization.Team = Initialization.AllianceColor.RED;
        Initialization.initializeLimeLight();
        OutTakeController.Initialize(gamepad1, gamepad2);
        IntakeController.Initialize(gamepad1, gamepad2);
        scheduler = new Scheduler();
        OutTakeStateMachine.ElevatorScoreSpecimen = 0;
        double prev = OutTakeStateMachine.ArmScoreSpecimen;
        OutTakeStateMachine.ArmScoreSpecimen = 90;

        scheduler
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
                        OutTakeStateMachine.ArmScoreSpecimen = prev;
                        return true;
                    }
                })
                .addTask(new GoToPoint(new Pose2d(scoreSpecimen.x, scoreSpecimen.y, scoreSpecimen.h)))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new GoToPoint(new Pose2d(presample1.x, presample1.y, presample1.h)))


        ;


        while (opModeInInit()){
            Elevator.update();
            Arm.update();
            Claw.close();
            OutTakeController.Update();
            IntakeController.Update(true);
        }

        while (isStarted()){

        }
    }
}
