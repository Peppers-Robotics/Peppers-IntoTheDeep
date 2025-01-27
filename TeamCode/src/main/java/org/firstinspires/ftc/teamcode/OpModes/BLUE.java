package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.Storage;

@TeleOp(name=".pippersBLUE ")
@Disabled
public class BLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MainOpMode op = new MainOpMode();
        op.hardwareMap = this.hardwareMap;
        op.gamepad1 = this.gamepad1;
        op.gamepad2 = this.gamepad2;
        Storage.team = Storage.Team.BLUE;
        op.runOpMode();
    }
}
