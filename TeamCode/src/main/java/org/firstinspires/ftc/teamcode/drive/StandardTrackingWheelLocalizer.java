package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.6229; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = PinPointLocalizer.X; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = PinPointLocalizer.Y; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.006993007; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0060488688; // Multiplier in the Y direction

    private PinPointLocalizer pinpointL;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE, 0), // left
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;
        pinpointL = new PinPointLocalizer(hardwareMap.get(PinPoint.class, "pinpoint"));

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "eM3"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "cM0"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = -pinpointL.pinPoint.getEncoderX();
        int frontPos = pinpointL.pinPoint.getEncoderY();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        double leftVel = pinpointL.pinPoint.getVelX();
        double frontVel = pinpointL.pinPoint.getVelY();

        lastEncVels.clear();
        lastEncVels.add((int)(leftVel * TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO)));
        lastEncVels.add((int)(frontVel * TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO)));

        return Arrays.asList(
                leftVel,
                frontVel
//                encoderTicksToInches(leftVel),
//                encoderTicksToInches(frontVel)
        );
    }

    @Override
    public double getHeading() {
        return pinpointL.getPoseEstimate().getHeading();
    }
}
