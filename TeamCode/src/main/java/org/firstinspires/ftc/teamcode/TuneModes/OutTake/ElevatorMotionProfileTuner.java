package org.firstinspires.ftc.teamcode.TuneModes.OutTake;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Initialization;
import org.firstinspires.ftc.teamcode.MainOpMode;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

import java.util.Vector;

@TeleOp(group = "OutTake")
@Config
public class ElevatorMotionProfileTuner extends LinearOpMode {
    public static double highBound = 1400;
    public static int repeatAMPtune = 2;
    private static double currentMaxVelocity = 0;

    private Vector<Double> maxVelocities, accelerations, decelerations;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Initialization.initializeRobot(hardwareMap);
        Initialization.UninitializeRobot();

        maxVelocities = new Vector<>();
        accelerations = new Vector<>();
        decelerations = new Vector<>();

        Thread t = new Thread(() -> {
            while (isStarted() && !isStopRequested()){
                Initialization.updateCacheing();
            }
        });

        waitForStart();
        t.start();
        telemetry.addLine("raise the elevator by hand to its maximum position");
        telemetry.addLine("press Y/▲ when the elevator is extended at its maximum");
        telemetry.update();

        while(!gamepad1.y);
        highBound = Elevator.motor.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Put the elevator down to its starting position (level 0) and press Y/▲ when done");
        telemetry.update();

        while (!gamepad1.y);

        while (repeatAMPtune -- > 0) {
            telemetry.clear();
            telemetry.addLine("Wait for max velocity, acceleration, decceleration determination");
            telemetry.update();
            Elevator.motor.setPower(1);
            double lastVelocity = 0;
            double acceleration = 0, deceleration = 0;
            ElapsedTime time = new ElapsedTime();
            while(Elevator.motor.getCurrentPosition() < highBound - 10){
                double currentAcceleration = (lastVelocity - Elevator.motor.getVelocity()) / time.seconds();
                if(currentAcceleration > acceleration) acceleration = currentAcceleration;
                if(currentAcceleration < deceleration) deceleration = currentAcceleration;


                if(Elevator.motor.getVelocity() > currentMaxVelocity) currentMaxVelocity = Elevator.motor.getVelocity();
                lastVelocity = Elevator.motor.getVelocity();
                time.reset();
            }
            Elevator.motor.setPower(0);

            telemetry.clear();
            telemetry.addData("Max velocity", currentMaxVelocity);
            telemetry.addData("Acceleration", acceleration);
            telemetry.addData("Deceleration", deceleration);
            telemetry.addLine("Put the elevator down to its starting position (level 0) and press Y/▲");
            telemetry.update();
            maxVelocities.add(currentMaxVelocity);
            accelerations.add(acceleration);
            decelerations.add(deceleration);
            currentMaxVelocity = 0;

            while (!gamepad1.y);

        }
        double mrv = 0, ar = 0, dr = 0;
        for(int i = 0; i < accelerations.size(); i++){
            mrv += maxVelocities.get(i);
            ar += accelerations.get(i);
            dr += decelerations.get(i);
        }
        mrv /= accelerations.size();
        ar /= accelerations.size();
        dr /= accelerations.size();
        telemetry.addData("Recommanded MaxVelocity", mrv);
        telemetry.addData("Recommanded Acceleration", ar);
        telemetry.addData("Recommanded Deceleration", dr);

        requestOpModeStop();
    }
}
