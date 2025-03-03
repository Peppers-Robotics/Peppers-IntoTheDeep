package org.firstinspires.ftc.teamcode.Autonomous;

import android.widget.TabHost;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.Extension;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

import kotlin.StandardKt;

@Autonomous(name = "0 + 7")
@Config
public class Sample extends LinearOpMode {
    public static boolean isRetracting = false;
    public static double parkElevator = 250;
    public static Limelight3A camera;
    private static LLResult result = null;
    public static class Transfer extends Task{
        private Scheduler transfer;
        public Transfer(){
            transfer = new Scheduler();
            {
                transfer
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                return Extendo.getCurrentPosition() < 20;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOn();
                                DropDown.setDown(0);
                                Extension.Extend(OutTakeLogic.TransferExtension);
                                return true;
                            }
                        })
                        .waitSeconds(0.05)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Claw.open();
                                ActiveIntake.Block();
                                Arm.setArmAngle(OutTakeLogic.ArmIdle-2);
                                DropDown.setDown(0);
                                Elevator.PowerOnDownToTakeSample = true;
                                Elevator.power = 1;
                                Extendo.PowerOnToTransfer = true;
                                Extendo.Extend(0);
                                return true;
                            }
                        })
                        .waitSeconds(0.10)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Claw.close();
                                return true;
                            }
                        })
                        .waitSeconds(0.05)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Elevator.PowerOnDownToTakeSample = false;
                                Extendo.PowerOnToTransfer = false;
                                Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                                if(Elevator.getTargetPosition() > 200)
                                    Arm.setArmAngle(OutTakeLogic.ArmUpSample);
                                return Arm.getCurrentArmAngle() >= 100;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOff();
//                                Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                                Extension.Extend(0.2);
                                return Math.abs(Elevator.getCurrentPosition() - Elevator.getTargetPosition()) < 300;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                                if(Arm.getCurrentArmAngle() >= 190){
                                    Extension.Extend(0);
                                }
                                DropDown.setDown(0);
                                return Arm.motionCompleted();
                            }
                        })
                ;
            }
        }
        @Override
        public boolean Run() {
            transfer.update();
            return transfer.done();
        }
    }
    public static class Retract extends Task{
        private Scheduler retract;
        public Retract(){
            retract = new Scheduler();
            retract
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            Claw.close();
                            if(Extendo.getTargetPosition() < 100)
                                Extendo.Extend(100);
//                            Extension.Extend(OutTakeLogic.TransferExtension);
                            Extension.Retract();
                            Arm.setArmAngle(OutTakeLogic.ArmIdle);
                            Elevator.PowerOnDownToTakeSample = true;
                            Elevator.setTargetPosition(0);
                            Elevator.power = 1;
                            isRetracting = true;
                            return true;
                        }
                    })
                    ;

        }

        @Override
        public boolean Run() {
            retract.update();
            return retract.done();
        }
    }
    private static int cnt = 0;
    public static class TakeSample extends Task {
        private Scheduler take;
        private double p = -1;
        private double tx, ty;
        private SparkFunOTOS.Pose2D took;
        public TakeSample(int pos, double timeOut){
            p = -1;
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            camera.start();
                            camera.pipelineSwitch(0);
                            Claw.open();
                            ActiveIntake.Unblock();
                            Extendo.Extend(0);
                            Chassis.setHeading(park.h);
                            Chassis.asyncFollow = false;
                            result = null;
                            return true;
                        }
                    })
                    .waitForSync()
//                    .waitForStill()
//                    .waitSeconds(0.2)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getAngleDifference(Localizer.getVelocity().h, 0) <= Math.toRadians(2) && Localizer.getAngleDifference(Localizer.getCurrentPosition().h, park.h) < Math.toRadians(1);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            Extension.Extend(OutTakeLogic.TransferExtension);
                            result = camera.getLatestResult();
                            took = Localizer.getCurrentPosition();
                            if(result != null && result.isValid()){
                                camera.captureSnapshot("image" + cnt++);
                            }
//                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y, park.h));
                            return result != null && result.isValid() && Localizer.getVelocity().x < 2 && Localizer.getVelocity().y < 2 && Localizer.getAngleDifference(park.h, Localizer.getCurrentPosition().h) < Math.toRadians(2);
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            LLResultTypes.DetectorResult res = GetPositionSample.getOptimalResult(result, 2);
//                            tx = res.getTargetXDegreesNoCrosshair();
//                            ty = res.getTargetYDegreesNoCrosshair();
                            tx = result.getTxNC();
                            ty = result.getTyNC();
                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(
                                    Localizer.getCurrentPosition().x,
                                    Localizer.getCurrentPosition().y,
                                    Localizer.getCurrentPosition().h +
                                    GetPositionSample.getExtendoRotPair(tx, ty).h
                            ));
                            return true;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Math.toDegrees(Math.abs(Localizer.getCurrentPosition().h - Chassis.getTargetPosition().h)) < 10;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(
                                    (int) GetPositionSample.getExtendoRotPair(
                                            tx, ty
                                    ).x - 130
                            );
                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 10;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.powerOn(0.7);
                            return true;
                        }
                    })

                    .addTask(new Task() {
                        boolean start = false;
                        long time = -1;
                        @Override
                        public boolean Run() {

                            Robot.telemetry.addData("target (ex, rot)", Extendo.getTargetPosition() + " " + Math.toRadians(Chassis.getTargetPosition().h));
                            Robot.telemetry.addData("sample distance forward", GetPositionSample.getPositionRelativeToRobot(tx, ty).x);
                            if(Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) <= 10 &&
                               Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= Math.toRadians(10)){
                                start = true;
                            }
                            if(start){
                                ActiveIntake.powerOn();
                                DropDown.setDown(1);
                                if(time == -1) time = System.currentTimeMillis();
                                if(p == -1) p = Extendo.getTargetPosition();
                            }
                            if(p != -1 && (System.currentTimeMillis() - time) / 1000.f >= 0.1 && start) {
                                Extendo.Extend(Extendo.getTargetPosition() + 800);
//                                Chassis.setHeading(Localizer.getCurrentPosition().h + Math.toRadians(10));
                            }
                            if(((System.currentTimeMillis() - time) / 1000.f > timeOut + (ActiveIntake.motor.getCurrent(CurrentUnit.AMPS) > 3 ? 0.5 : 0) && time != -1)){
                                return true;
                            }
                            Robot.telemetry.addData("POW", ActiveIntake.motor.getCurrent(CurrentUnit.AMPS));
                            return Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.hasTeamPice()) {
                                DropDown.setDown(0);
                                Extension.Extend(OutTakeLogic.TransferExtension);
                                result = null;
                                return true;
                            } else {
                                ActiveIntake.Block();
                            }
                            result = null;
                            camera.reloadPipeline();
                            return true;
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            if(Storage.hasTeamPice())
                                ActiveIntake.Block();
                            ActiveIntake.Reverse(0.5);
                            Extendo.Extend(0);
                            DropDown.setDown(0);
                            return true;
                        }
                    })
            ;
        }
        public TakeSample(int pos) {
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            DropDown.setDown(1);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > pos - 20 || Storage.hasTeamPice();
                        }
                    })
                    .addTask(new Task() {
                        private long time = -1;
                        @Override
                        public boolean Run() {
                            if(time == -1){
                                time = System.currentTimeMillis();
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= 0.2){
                                Extendo.Extend(pos + 100);
                            }
                            return Storage.hasTeamPice() || (double) (System.currentTimeMillis() - time) / 1000.f >= 0.7;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extension.Extend(OutTakeLogic.TransferExtension);
                            ActiveIntake.Unblock();
//                            ActiveIntake.Block();
                            ActiveIntake.powerOn();
                            Extendo.Extend(0);
                            DropDown.setDown(0);
                            return true;
                        }
                    })
                    ;
        }
        @Override
        public boolean Run() {
            take.update();
            return take.done();
        }
    }
    public static class GoToTakeSampleFromSubmersible extends Task{
        private Scheduler go, tmp;
        private static final Scheduler takeTry = new Scheduler();
        static {
            takeTry.
                    addTask(new TakeSample(0, 0.4))
                    ;
        }
        public GoToTakeSampleFromSubmersible(){
            go = new Scheduler();
            go
                    .lineToLinearHeadingAsync(park)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            return Localizer.getCurrentPosition().y < -250;
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp = new Scheduler();
                            tmp.addTask(new TakeSample(0, 0.5));
//                            Extension.Retract();
                            Extension.Extend(OutTakeLogic.TransferExtension);
                            return true;
                        }
                    })
                    .addTask(new Retract())
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extendo.Extend(0);
                            return true;
                        }
                    })
                    .waitForTrajDone(90)
                    .lineTo(new SparkFunOTOS.Pose2D(park.x, park.y + 100, park.h))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            tmp.update();
                            if(tmp.done() && !Storage.hasTeamPice()){
                                tmp.addTask(new TakeSample(0, 0.5));
                            }
                            return tmp.done();
                        }
                    })
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.setProfiles(3000, 3000, 4500, 4500, 700, 700);
                            return true;
                        }
                    })
                    .lineToLinearHeadingAsync(basketPosition)
                    .addTask(new Transfer())
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                            Chassis.setProfiles(4000, 4000, 3000, 3000, 700, 700);
                            return true;
                        }
                    })
                    ;
        }

        @Override
        public boolean Run() {
            go.update();
            return go.done();
        }
    }

    public static SparkFunOTOS.Pose2D
            basketPosition = new SparkFunOTOS.Pose2D(500, -100, Math.toRadians(48)),
            sample1 = new SparkFunOTOS.Pose2D(510, -300, Math.toRadians(70)),
            sample2 = new SparkFunOTOS.Pose2D(510, -230, Math.toRadians(78)),
            sample3 = new SparkFunOTOS.Pose2D(470, -260, Math.toRadians(110)),
            park = new SparkFunOTOS.Pose2D(-420, -1450, Math.toRadians(20)),
            parkk = new SparkFunOTOS.Pose2D(-470, -1460, Math.toRadians(0))
    ;

    private static long startTime = 0;
    private static double autoTimer = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Extendo.Extend(0);
        DropDown.setDown(0);
        Arm.setArmAngle(50);
        Elevator.setTargetPosition(0);
        Claw.close();
        Chassis.resetProfiles();
        camera = hardwareMap.get(Limelight3A.class, "camera");
        camera.start();
        camera.pipelineSwitch(0);
        Extension.Retract();
        Chassis.setProfiles(6500, 6500, 10000, 10000, 4000, 4000);
//        Storage.team = Storage.Team.RED;
        cnt = 0;
        camera.deleteSnapshots();
        parked = false;

        Scheduler scheduler = new Scheduler();
        scheduler
                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x, basketPosition.y, basketPosition.h))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
                        Arm.setPivotAngle(10);
                        Arm.setArmAngle(220);
                        return true;
                    }
                })
//                .lineTo(basketPosition)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Elevator.getCurrentPosition() > OutTakeLogic.ElevatorScoreSample2 - 150;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                        return Arm.motionCompleted();
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extension.Extend(0.2);
//                        Extension.Retract();
                        DropDown.setDown(1);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        ActiveIntake.powerOn();
                        return true;
                    }
                })
                .waitSeconds(0.08)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        return true;
                    }
                })
                .lineToAsync(sample1)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        DropDown.setDown(0.4);
                        return Localizer.getCurrentPosition().h > Math.toRadians(45) && Localizer.getCurrentPosition().y < -200;
                    }
                })
                .addTask(new Retract())
//                .waitForSync()
                .addTask(new TakeSample(850))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
                        ActiveIntake.powerOn();
                        return true;
                    }
                })
                .waitForSync()
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        return true;
                    }
                })
//                .waitSeconds(0.05)
                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(0);
//                        return Localizer.getCurrentPosition().y < -200;
                        return Localizer.getCurrentPosition().h > Math.toRadians(50) && Localizer.getCurrentPosition().y < -200;
                    }
                })
                .addTask(new Retract())
//                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        DropDown.setDown(0.2);
                        Extendo.Extend(650);
                        return Localizer.getCurrentPosition().h > Math.toRadians(80);
                    }
                })
                .addTask(new TakeSample(850))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        return true;
                    }
                })
                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
//                        return Localizer.getCurrentPosition().y < -200;
                        Extendo.Extend(500);
                        ActiveIntake.powerOn();
                        DropDown.setDown(0.9);
                        return Localizer.getCurrentPosition().h > Math.toRadians(80);
                    }
                })
                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(600))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .waitSeconds(0.1)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                        Chassis.setProfiles(4000, 4000, 3000, 3000, 700, 700);
                        return true;
                    }
                })
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                .addTask(new GoToTakeSampleFromSubmersible())
                ;

        DropDown.setDown(0);
        while(opModeInInit()){
            Robot.clearCache(false);
            if(gamepad1.square) Storage.team = Storage.Team.RED;
            if(gamepad1.circle) Storage.team = Storage.Team.BLUE;

            telemetry.addData("TEAM", Storage.team.toString());
            Extendo.update();
            Arm.update();
            Elevator.update();
            telemetry.update();
        }
        startTime = System.currentTimeMillis();
        autoTimer = 0;
        long auto = System.currentTimeMillis();
        LinearFunction f = new LinearFunction(0.7, 2);

        while(opModeIsActive()){
            Robot.clearCache();
            if(isRetracting && Elevator.getCurrentPosition() < 10) Elevator.PowerOnDownToTakeSample = false;
            if(autoTimer >= 30 - f.getOutput(Math.abs((Math.abs(park.x) - Math.abs(Localizer.getCurrentPosition().x)) / Math.abs(park.x))) && !parked){
                parked = true;
                scheduler = new Scheduler();
                scheduler
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Chassis.setTargetPosition(parkk);
                                Chassis.asyncFollow = false;
                                Elevator.setTargetPosition(parkElevator);
//                                if(Elevator.getTargetPosition() > Elevator.getTargetPosition() - 120)
                                    Arm.setArmAngle(110);
                                return Elevator.getCurrentPosition() > Elevator.getTargetPosition() - 5;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Extension.Extend(1);
                                return true;
                            }
                        })

                        ;

            } else {
                scheduler.update();
            }

            Robot.telemetry.addData("timer", (System.currentTimeMillis() - auto) / 1000.f);
            telemetry.addData("color", Storage.getStorageStatus().toString());
            autoTimer = (System.currentTimeMillis() - auto) / 1000.f;
            if((System.currentTimeMillis() - auto) / 1000.f >= 30) requestOpModeStop();
            Extendo.update();
            Localizer.Update();
            Chassis.Update();
            Arm.update();
            Elevator.update();
            telemetry.update();
        }
    }
    public static boolean parked = false;
}