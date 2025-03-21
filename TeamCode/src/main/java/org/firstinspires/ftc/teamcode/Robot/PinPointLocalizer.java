package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.ftc.bumblebee.Localizers.Localizer;
import com.ftc.bumblebee.Localizers.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.PinPoint;

@Config
@SuppressWarnings("unused")
public class PinPointLocalizer extends Localizer {
    public double Xoffset = 118.016, Yoffset = 0.116;
    public static PinPoint device;
    public static PinPoint.EncoderDirection xDir = PinPoint.EncoderDirection.FORWARD, yDir = PinPoint.EncoderDirection.FORWARD;
    //ticks / mm
    public static double encoderResolution = 19.89436789f;
    public PinPointLocalizer(PinPoint p){
        device = p;
        device.setEncoderDirections(xDir, yDir);
        device.setEncoderResolution(encoderResolution);
    }
    @Override
    public Pose2d getCurrentPosition() {
        double x = device.getPosX();
        double y = device.getPosY();
        double h = Pose2d.NormalizeRadians(device.getHeading());
        return new Pose2d(x, y, h);
    }

    public void update(){
        device.update();
    }

    @Override
    public Pose2d getCurrentelocity() {
        double x = device.getVelX();
        double y = device.getVelY();
        double h = device.getHeadingVelocity();
        return new Pose2d(x, y, h);
    }

    public void beaconUpdate(Pose2d measurement, DistanceUnit unit) {
        measurement.h = Pose2d.NormalizeRadians(measurement.h);
        device.setPosition(new Pose2D(unit, measurement.x, measurement.y, AngleUnit.RADIANS, measurement.h));
    }
}
