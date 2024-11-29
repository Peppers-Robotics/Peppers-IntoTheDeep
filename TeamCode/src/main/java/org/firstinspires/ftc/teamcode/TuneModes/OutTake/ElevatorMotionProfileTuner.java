package org.firstinspires.ftc.teamcode.TuneModes.OutTake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.AutoGamepad;
import org.firstinspires.ftc.teamcode.Initialization;
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
        AutoGamepad gamepad = new AutoGamepad(gamepad1);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Initialization.initializeHubCacheing(hardwareMap);
        Initialization.initializeElevator();

        maxVelocities = new Vector<>();
        accelerations = new Vector<>();
        decelerations = new Vector<>();

        waitForStart();
        telemetry.addLine("raise the elevator by hand to its maximum position");
        telemetry.addLine("press Y/▲ when the elevator is extended at its maximum");
        telemetry.update();

        do {
            gamepad.update();
        } while (!gamepad.wasPressed.y);
        Initialization.updateCacheing();
        highBound = Elevator.motor.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Put the elevator down to its starting position (level 0) and press Y/▲ when done");
        telemetry.update();

        do {
            gamepad.update();
        } while (!gamepad.wasPressed.y);

        while (repeatAMPtune -- > 0) {
            telemetry.clear();
            telemetry.addLine("Wait for max velocity, acceleration, decceleration determination");
            telemetry.update();
            double lastVelocity = 0;
            double acceleration = 0, deceleration = 0;
            ElapsedTime time = new ElapsedTime();
            Elevator.motor.setPower(-1);
            while(Elevator.motor.getCurrentPosition() < highBound - 50){
                telemetry.addData("Elevator current position", Elevator.motor.getCurrentPosition());
                Initialization.updateCacheing();
                double currentAcceleration = (lastVelocity - Elevator.motor.getVelocity()) / -time.seconds();
                if(currentAcceleration > acceleration) acceleration = currentAcceleration;
                if(currentAcceleration < deceleration) deceleration = currentAcceleration;


                if(Elevator.motor.getVelocity() > currentMaxVelocity) currentMaxVelocity = Elevator.motor.getVelocity();
                lastVelocity = Elevator.motor.getVelocity();
                time.reset();
                telemetry.update();
            }
            Elevator.motor.setPower(0);

            telemetry.clear();
            telemetry.addData("Max velocity", currentMaxVelocity);
            telemetry.addData("Acceleration", acceleration);
            telemetry.addData("Deceleration (non accurate)", deceleration);
            telemetry.addLine("Put the elevator down to its starting position (level 0) and press Y/▲");
            telemetry.update();
            maxVelocities.add(currentMaxVelocity);
            accelerations.add(acceleration);
            decelerations.add(deceleration);
            currentMaxVelocity = 0;

            do {
                gamepad.update();
            } while (!gamepad.wasPressed.y);

        }
        double mrv = 0, ar = 0, dr = 0;
        for(int i = 0; i < accelerations.size(); i++){
            mrv += maxVelocities.get(i);
            ar += accelerations.get(i);
            dr += decelerations.get(i);
        }
        mrv /= maxVelocities.size();
        ar /= accelerations.size();
        dr /= decelerations.size();
        telemetry.clear();
        telemetry.addData("Recommanded MaxVelocity", mrv);
        telemetry.addData("Recommanded Acceleration", ar);
        telemetry.addData("Recommanded Deceleration (recommandation: -acceleration)", dr);
        telemetry.update();

        while (opModeIsActive());
    }
}
