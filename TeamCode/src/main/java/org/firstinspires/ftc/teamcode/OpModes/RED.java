package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.Storage;

@TeleOp(name = ".peppersRED ")
@Disabled
public class RED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MainOpMode op = new MainOpMode();
        Storage.team = Storage.Team.RED;
        op.hardwareMap = this.hardwareMap;
        op.gamepad1 = this.gamepad1;
        op.gamepad2 = this.gamepad2;

        op.runOpMode();
    }
}
