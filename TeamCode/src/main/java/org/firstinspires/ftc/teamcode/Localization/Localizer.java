package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

abstract class Localizer {
    protected Pose2D currentPosition, lastPosition;

    abstract public void update();
    abstract public void beaconUpdate(Pose2D pose);

    public Pose2D getCurrentPosition(){
        return currentPosition;
    }
}
