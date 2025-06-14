package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
public class TestRotExt extends LinearOpMode {
    public static LLResult res;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        DropDown.setDown(0);
        Extendo.Extend(0);
        Chassis.Forward.pidCoefficients = new PIDCoefficients();
        Chassis.Strafe.pidCoefficients = new PIDCoefficients();


        Limelight3A ll = hardwareMap.get(Limelight3A.class, "camera");
        ll.start();
        ll.pipelineSwitch(0);
        res = null;
        SparkFunOTOS.Pose2D capture = new SparkFunOTOS.Pose2D(), p, o;

        waitForStart();

        while(opModeIsActive()){
            if(res == null && gamepad1.square){
                res = ll.getLatestResult();
                capture = Localizer.getCurrentPosition();
            }
            else if(res != null) {
                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0, 0,
                        GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture), Localizer.getCurrentPosition()).h
                        ));
                Extendo.Extend((int) GetPositionSample.getExtendoRotPairByField(GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture), Localizer.getCurrentPosition()).x);
                p = GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture);
                o = GetPositionSample.getPositionRelativeToRobot(res.getTx(), res.getTy());
                Robot.telemetry.addData("field pos", "(" + p.x + ", " + p.y + ", " + Math.toDegrees(p.h) + " deg)");
                Robot.telemetry.addData("robot pos", "(" + o.x + ", " + o.y + ", " + Math.toDegrees(o.h) + " deg)");
                Robot.telemetry.addData("extendo dist", Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), GetPositionSample.getPositionRelativeToFiled(res.getTx(), res.getTy(), capture)));
            }
            Robot.telemetry.addData("res null?", res == null);
            Robot.clearCache();
            Localizer.Update();
            if(res != null)
                Chassis.Update();
            Extendo.update();
        }

    }
}
