package org.firstinspires.ftc.teamcode.Autonomous;

import android.widget.TabHost;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.GetPositionSample;
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

@Autonomous(name = "0 + 7")
@Config
public class Sample extends LinearOpMode {
    public static boolean isRetracting = false;
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
                                Arm.setArmAngle(OutTakeLogic.ArmTransfer);
                                DropDown.setDown(0);
                                Elevator.PowerOnDownToTakeSample = true;
                                Elevator.power = 1;
                                Extendo.PowerOnToTransfer = true;
                                Extendo.Extend(0);
                                return true;
                            }
                        })
                        .waitSeconds(0.05)
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
                                Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2 + 50);
                                if (Elevator.getCurrentPosition() > OutTakeLogic.ElevatorUp - 80) {
                                    Arm.setArmAngle(OutTakeLogic.ArmUpSample);
//                                    Arm.setPivotAngle(OutTakeLogic.PivotUpSample);
                                }
                                return Arm.getCurrentArmAngle() >= 100;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOff();
//                                Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                                return Math.abs(Elevator.getCurrentPosition() - OutTakeLogic.ElevatorScoreSample2) < 300;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
                                if(Arm.getCurrentArmAngle() >= 190){
                                    Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                                }
                                DropDown.setDown(0);
                                return Arm.motionCompleted();
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                                return true;
                            }
                        })
//                        .waitSeconds(0.1)
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
                            Claw.close();
                            Extendo.Extend(600);
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
    public static class TakeSample extends Task {
        private Scheduler take;
        private SparkFunOTOS.Pose2D took;
        public TakeSample(int pos, double timeOut){
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Claw.open();
                            ActiveIntake.Unblock();
                            Extendo.Extend(0);
                            result = null;
                            return true;
//                            return Extendo.getCurrentPosition() > pos - 10;
                        }
                    })
//                    .waitForSync()
//                    .lineToAsync(new SparkFunOTOS.Pose2D(park.x, park.y - 100, park.h))
//                    .waitForSync()
//                    .waitSeconds(0.6)
//                    .addTask(new Task() {
//                        @Override
//                        public boolean Run() {
//                            return Chassis.getPrecentageOfMotionDone() > 90;
//                        }
//                    })
                    .waitForSync()
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            result = camera.getLatestResult();
                            took = Localizer.getCurrentPosition();
                            if(result != null){
                                new Thread(() -> camera.reloadPipeline()).start();
                            }
                            return result != null;
                        }
                    })
//                    .addTask(new Task() {
//                        @Override
//                        public boolean Run() {
//                            return Chassis.getPrecentageOfMotionDone() > 98;
//                        }
//                    })
                    .waitForSync()

                    .addTask(new Task() {
                        boolean start = false;
                        long time = -1;
                        @Override
                        public boolean Run() {
                            Extendo.Extend(
//                                    (int) GetPositionSample.getExtendoRotPairByField(
//                                            GetPositionSample.getPositionRelativeToFiled(result.getTx(), result.getTy(), took)
//                                    ).x
                                    0
                            );
                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(
                                    park.x,
                                    park.y,
                                    GetPositionSample.getExtendoRotPairByField(
                                            GetPositionSample.getPositionRelativeToFiled(result.getTx(), result.getTy(), took)
                                    ).h
                            ));
                            Robot.telemetry.addData("target (ex, rot)", Extendo.getTargetPosition() + " " + Chassis.getTargetPosition().h);
                            if(Math.abs(Extendo.getCurrentPosition() - Extendo.getTargetPosition()) <= 20 &&
                               Localizer.getAngleDifference(Localizer.getCurrentPosition().h, Chassis.getTargetPosition().h) <= 10){
                                start = true;
                            }
                            if(start){
                                ActiveIntake.powerOn();
                                DropDown.setDown(1);
                                if(time == -1) time = System.currentTimeMillis();
                            }
                            return Storage.getStorageStatus() == Storage.SpecimenType.YELLOW || ((System.currentTimeMillis() - time) / 1000.f > timeOut && time != -1);
//                            return Extendo.getCurrentPosition() > Extendo.getTargetPosition() - 20;
                        }
                    })

                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Extension.Extend(OutTakeLogic.TransferExtension);
                            ActiveIntake.Block();
                            result = null;
                            return true;
                        }
                    })
                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                            park = new SparkFunOTOS.Pose2D(park.x, park.y + 10, park.h + Math.toRadians(10));
                            ActiveIntake.Block();
                            ActiveIntake.Reverse(0.8);
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
        private Scheduler go;
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
                            Extension.Retract();
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
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
//                        Chassis.setProfiles(3000, 3000, 1500, 1500, 500, 500);
                            return Chassis.getPrecentageOfMotionDone() > 80;
                        }
                    })
//                .waitForSync()
                    .addTask(new TakeSample(700, 0.4))
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            Chassis.setProfiles(2000, 2000, 2000, 2000, 700, 700);
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
            basketPosition = new SparkFunOTOS.Pose2D(510, -120, Math.toRadians(48)),
            basketPosition2 = new SparkFunOTOS.Pose2D(0, 0, 0),
            basketPosition3 = new SparkFunOTOS.Pose2D(0, 0 ,0),
            basketPosition4 = new SparkFunOTOS.Pose2D(0, 0, 0),
            basketPositionSub = new SparkFunOTOS.Pose2D(0, 0, 0),

            sample1 = new SparkFunOTOS.Pose2D(510, -200, Math.toRadians(68)),
            sample2 = new SparkFunOTOS.Pose2D(510, -220, Math.toRadians(84)),
            sample3 = new SparkFunOTOS.Pose2D(470, -260, Math.toRadians(110)),
            park = new SparkFunOTOS.Pose2D(-380, -1350, Math.toRadians(0)),
            parkk = new SparkFunOTOS.Pose2D(-470, -1460, Math.toRadians(0))
    ;

    private static long startTime = 0;
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
        Chassis.setProfiles(6000, 6000, 4000, 4000, 1200, 1200);
        Storage.team = Storage.Team.BLUE;
        Robot.telemetry = new MultipleTelemetry(telemetry, Robot.telemetry);

        Scheduler scheduler = new Scheduler();
        scheduler
                .lineToAsync(new SparkFunOTOS.Pose2D(basketPosition.x, basketPosition.y + 30, basketPosition.h))
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2 + 50);
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
                        Extension.Extend(OutTakeLogic.ScoreSampleExtension);
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Extendo.Extend(850);
                        ActiveIntake.powerOn();
                        DropDown.setDown(1);
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
                        DropDown.setDown(0);
                        return Localizer.getCurrentPosition().h > Math.toRadians(60);
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
                        Extendo.Extend(600);
                        ActiveIntake.powerOn();
                        return true;
                    }
                })
                .waitForSync()
                .waitSeconds(0.05)
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
                        Extendo.Extend(850);
//                        return Localizer.getCurrentPosition().y < -200;
                        return Localizer.getCurrentPosition().h > Math.toRadians(50);
                    }
                })
                .addTask(new Retract())
//                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().h > Math.toRadians(70);
                    }
                })
                .addTask(new TakeSample(850))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .waitSeconds(0.05)
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
                        return Localizer.getCurrentPosition().h > Math.toRadians(50);
                    }
                })

                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(835))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .waitSeconds(0.05)

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




        while(opModeInInit()){
            Robot.clearCache();

            Extendo.update();
            Arm.update();
            Elevator.update();
        }
        startTime = System.currentTimeMillis();
        long auto = System.currentTimeMillis();

        while(opModeIsActive()){
            Robot.clearCache();
            if(isRetracting && Elevator.getCurrentPosition() < 10) Elevator.PowerOnDownToTakeSample = false;

            Robot.telemetry.addData("timer", (System.currentTimeMillis() - auto) / 1000.f);
            if((System.currentTimeMillis() - auto) / 1000.f >= 30) requestOpModeStop();
            scheduler.update();
            Extendo.update();
            Localizer.Update();
            Chassis.Update();
            Arm.update();
            Elevator.update();
        }
    }
}