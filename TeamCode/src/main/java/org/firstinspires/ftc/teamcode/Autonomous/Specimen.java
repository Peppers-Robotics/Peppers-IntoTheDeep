package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HelperClasses.Pose2D;
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
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Autonomous
@Config
@Disabled
public class Specimen extends LinearOpMode {
    public static int Extendo1 = 0, Extendo2 = 10, Extendo3 = 20,
                         ExtendoRev1 = 10, ExtendoRev2 = 20, ExtendoRev3 = 10;
    public static double dropDown = 0.6;
    public static Pose2D scoreSpecimen = new Pose2D(0, 0, 0),
    presample1 = new Pose2D(0, 0, 0),
    sample1 = new Pose2D(0, 0, 0),
    sample2 = new Pose2D(0, 0, 0),
    sample3 = new Pose2D(0, 0, 0),
    pretakeSpecimen = new Pose2D(0, 0 ,0),
    takeSpecimen = new Pose2D(0, 0, 0),
    human1 = new Pose2D(0, 0, 0),
    human2 = new Pose2D(0, 0, 0),
    human3 = new Pose2D(0, 0, 0),
    preWallhuman = new Pose2D(0, 0, 0),
    preWall = new Pose2D(0, 0, 0);

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
                .addTask(new Task() { // extend to score specimen 1
                    @Override
                    public boolean Run() {
                        OutTakeStateMachine.ElevatorScoreSpecimen = OutTakeStateMachine.ElevatorSpecimen2;
                        OutTakeStateMachine.ArmScoreSpecimen = prev;
                        return true;
                    }
                })
                .goTo(scoreSpecimen)
                .addTask(new Task() { // score specimen 1
                    @Override
                    public boolean Run() {
                        OutTakeStateMachine.Update(OutTakeStateMachine.OutTakeActions.SCORE);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .goTo(presample1)
                .addTask(new Task() { // take sample
                    @Override
                    public boolean Run() {
                        Extendo.Extend(Extendo1, 30);
                        DropDown.setInstantPosition(dropDown);
                        ActiveIntake.powerOn();
                        return true;
                    }
                })
                .goToWithoutBlock(sample1)
                .addTask(new Task() { // wait for sample
                    @Override
                    public boolean Run() {
                        return !Storage.isStorageEmpty();
                    }
                })
                .addTask(new Task() { // prepare reverse
                    @Override
                    public boolean Run() {
                        Extendo.Extend(ExtendoRev1);
                        DropDown.setInstantPosition(0);
                        return true;
                    }
                })
                .goTo(human1)
                .addTask(new Task() { // reverse sample
                    @Override
                    public boolean Run() {
                        ActiveIntake.Reverse();
                        ActiveIntake.UnblockIntake();
                        return true;
                    }
                })
                .waitSeconds(0.2)

                .goTo(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(Extendo2);
                        ActiveIntake.powerOn();
                        DropDown.setInstantPosition(dropDown);
                        return !Storage.isStorageEmpty();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return false;
                    }
                })
                .goTo(human2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return true;
                    }
                })

        ;


        while (opModeInInit()){
            Elevator.update();
            Arm.update();
            Claw.close();
            OutTakeController.Update();
            IntakeController.Update();
        }
        Extendo.Extend(0);

        while (isStarted()){


            OutTakeStateMachine.Update(null);
            Elevator.update();
            Arm.update();
            Claw.close();
        }
    }
}
