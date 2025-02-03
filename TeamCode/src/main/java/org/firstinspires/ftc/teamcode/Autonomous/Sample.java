package org.firstinspires.ftc.teamcode.Autonomous;

import android.widget.TabHost;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Intake.DropDown;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OutTake.Arm;
import org.firstinspires.ftc.teamcode.OutTake.Claw;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;
import org.firstinspires.ftc.teamcode.OutTake.OutTakeLogic;
import org.firstinspires.ftc.teamcode.Robot.Chassis;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Tasks.Scheduler;
import org.firstinspires.ftc.teamcode.Tasks.Task;

@Autonomous(name = "0 + 4")
@Config
public class Sample extends LinearOpMode {
    public static boolean isRetracting = false;
    public static class Transfer extends Task{
        private Scheduler transfer;
        public Transfer(){
            transfer = new Scheduler();
            {
                transfer
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                return Extendo.getCurrentPosition() < 30;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOn();
                                DropDown.setDown(0.2);
                                return true;
                            }
                        })
                        .waitSeconds(0.1)
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Claw.open();
                                ActiveIntake.Block();
                                Arm.setArmAngle(OutTakeLogic.ArmTransfer);
                                DropDown.setDown(0.2);
                                Elevator.PowerOnDownToTakeSample = true;
                                Elevator.power = 1;
                                Extendo.PowerOnToTransfer = true;
                                Extendo.Extend(0);
                                return true;
                            }
                        })
                        .waitSeconds(0.2)
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
                                if (Elevator.getCurrentPosition() > OutTakeLogic.ElevatorUp - 80) {
                                    Arm.setArmAngle(OutTakeLogic.ArmUpSample);
                                    Arm.setPivotAngle(OutTakeLogic.PivotUpSample);
                                }
                                return Arm.getCurrentArmAngle() >= 100;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                ActiveIntake.powerOff();
                                return Math.abs(Elevator.getCurrentPosition() - OutTakeLogic.ElevatorScoreSample2) < 300;
                            }
                        })
                        .addTask(new Task() {
                            @Override
                            public boolean Run() {
                                Arm.setArmAngle(OutTakeLogic.ArmScoreSample);
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
                            Extendo.Extend(400);
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
    public static class TakeSample extends Task{
        private Scheduler take;
        public TakeSample(int pos, double timeOut){
            take = new Scheduler();
            take
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > pos - 10;
                        }
                    })
                    .addTask(new Task() {
                        private long time = -1;
                        @Override
                        public boolean Run() {
                            DropDown.setDown(1);
                            if(time == -1){
                                time = System.currentTimeMillis();
                            }
                            if(ActiveIntake.motor.getPower() > 0 && (System.currentTimeMillis() - time) / 1000.f > 0.15){
                                ActiveIntake.powerOn(1);
                                DropDown.setDown(1);
                            }
                            if((System.currentTimeMillis() - time) / 1000.f >= timeOut + 0.05){
                                Extendo.Extend(Extendo.getCurrentPosition() + 200);
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x - 100, Localizer.getCurrentPosition().y + 10, Localizer.getCurrentPosition().h + Math.toRadians(10)));
                                park = Chassis.getTargetPosition();
                                ActiveIntake.Reverse(1);
                                DropDown.setDown(0.5);
                                time = -1;
                            }
                            return Storage.hasTeamPice();
                        }
                    })
                    .waitSeconds(0.1)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            ActiveIntake.Block();
                            return true;
                        }
                    })
                    .waitSeconds(0.05)
                    .addTask(new Task() {
                        @Override
                        public boolean Run() {
                            park = new SparkFunOTOS.Pose2D(park.x - 100, park.y + 10, park.h + Math.toRadians(10));
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
                            ActiveIntake.Unblock();
                            ActiveIntake.powerOn(1);
                            DropDown.setDown(1);
                            Extendo.Extend(pos);
                            return Extendo.getCurrentPosition() > pos - 10;
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
    public static SparkFunOTOS.Pose2D
            basketPosition = new SparkFunOTOS.Pose2D(514, -147, Math.toRadians(47)),
            sample1 = new SparkFunOTOS.Pose2D(280, -345, Math.toRadians(85)),
            sample2 = new SparkFunOTOS.Pose2D(450, -400, Math.toRadians(95)),
            sample3 = new SparkFunOTOS.Pose2D(459, -400, Math.toRadians(115)),
            park = new SparkFunOTOS.Pose2D(-200, -1300, Math.toRadians(13)),
            parkk = new SparkFunOTOS.Pose2D(-470, -1460, Math.toRadians(0)),
            C = new SparkFunOTOS.Pose2D(460, 480, 0),
            D = new SparkFunOTOS.Pose2D(-50, 724, 0)
    ;


    private static long startTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.InitializeFull(hardwareMap);
        Robot.enable();
        Extendo.Extend(0);
        DropDown.setDown(0);
        Arm.setArmAngle(90);
        Elevator.setTargetPosition(0);
        Claw.close();
        Chassis.resetProfiles();

        Scheduler scheduler = new Scheduler();
        scheduler
                .lineToAsync(basketPosition)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(OutTakeLogic.ElevatorScoreSample2);
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
                .waitForSync()
                .waitSeconds(0.3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .lineToAsync(sample1)

                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -250;
                    }
                })

                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(750))
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
//                .waitSeconds(0.05)
                .lineToAsync(sample2)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -200;
                    }
                })
                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(650))
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
                .waitSeconds(0.05)
                .lineToAsync(sample3)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -200;
                    }
                })

                .addTask(new Retract())
                .waitForSync()
                .addTask(new TakeSample(650))
                .lineToAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .waitSeconds(0.05)


                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
                        Chassis.setProfiles(2000, 2000, 2000, 2000, 700, 700);
                        return true;
                    }
                })

                .lineToLinearHeadingAsync(park)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -300;
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
                        return Chassis.getPrecentageOfMotionDone() > 90;
                    }
                })
                .addTask(new TakeSample(700, 0.5))
//                .waitForSync()
                .lineToLinearHeadingAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
//                        Chassis.setProfiles(1000, 1000, 700, 700, 500, 500);
                        return true;
                    }
                })

                .lineToLinearHeadingAsync(park)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -300;
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
                        return Chassis.getPrecentageOfMotionDone() > 90;
                    }
                })
//                .waitForSync()
                .addTask(new TakeSample(800, 0.5))
                .lineToLinearHeadingAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
//                        Chassis.setProfiles(1000, 1000, 700, 700, 500, 500);
                        return true;
                    }
                })

                .lineToLinearHeadingAsync(park)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -300;
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
                        return Chassis.getPrecentageOfMotionDone() > 90;
                    }
                })
//                .waitForSync()
                .addTask(new TakeSample(800, 0.5))
                .lineToLinearHeadingAsync(basketPosition)
                .addTask(new Transfer())
                .waitForSync()
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Claw.open();
                        Robot.telemetry.addData("time", (System.currentTimeMillis() - startTime) / 1000.f);
//                        Chassis.setProfiles(1000, 1000, 700, 700, 500, 500);
                        Chassis.resetProfiles();
                        return true;
                    }
                })
                .lineToLinearHeadingAsync(parkk)
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        return Localizer.getCurrentPosition().y < -300;
                    }
                })
                .addTask(new Task() {
                    @Override
                    public boolean Run() {
                        Elevator.setTargetPosition(400);
                        Arm.setArmAngle(120);
                        return false;
                    }
                })
                .waitForSync()
                .lineTo(parkk)

//                .lineToAsync(new SparkFunOTOS.Pose2D(park.x, park.y, basketPosition.h))
//                .lineToAsync(park)
//                .waitSeconds(0.4)
//                .addTask(new Retract())
//                .addTask(new Task() {
//                    @Override
//                    public boolean Run() {
//                        Extendo.Extend(0);
//                        return true;
//                    }
//                })
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