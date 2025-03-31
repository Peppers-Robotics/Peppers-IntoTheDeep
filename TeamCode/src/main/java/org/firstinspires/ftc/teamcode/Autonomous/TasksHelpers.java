package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

import java.util.Locale;

public class TasksHelpers {
    public static class TakeSampleDetection extends Task {
        private final Scheduler s;
        private LLResult res;
        private SparkFunOTOS.Pose2D capture, posInField;
        long time = -1;
        public TakeSampleDetection(int id, Limelight3A camera){
            s = new Scheduler();
            s
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            res = camera.getLatestResult();
                            return res != null && res.isValid() && GetPositionSample.hasId(res, id);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            capture = Localizer.getCurrentPosition();
                            posInField = GetPositionSample.getPositionRelativeToFiled(
                                    GetPositionSample.getOptimalResult(res, id).getTargetXDegrees(), GetPositionSample.getOptimalResult(res, id).getTargetYDegrees(), Localizer.getCurrentPosition());
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.setHeading(GetPositionSample.getExtendoRotPairByField(posInField, Localizer.getCurrentPosition()).h);
                            Extendo.Extend((int) GetPositionSample.getExtendoRotPairByField(posInField, Localizer.getCurrentPosition()).x - 35);
                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 20 && Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) < Math.toRadians(3);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);

                            return false;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(time == -1) time = System.currentTimeMillis();
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.2) Extendo.Extend(Extendo.getTargetPosition() + 200);
                            return Storage.getStorageStatus() == GetPositionSample.getType(id) || (System.currentTimeMillis() - time) / 1000.f >= 0.5;
                        }
                    })
                    ;
        }
        @Override
        public boolean Run() {
            s.update();
            return s.done();
        }
    }
}
