package org.firstinspires.ftc.teamcode.Localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;
import org.firstinspires.ftc.teamcode.Initialization;

@Config
public class ThreeDeadWheelLocalizer extends Localizer{
    // TODO: tune this
    public static double LeftSign = -1, RightSign = 1, PerpendicularSign = 1; // reverse if needed
    public static double wheelDiameter = 32, lateralDistance = 26.38, perpendicularDistance = 10;
    public static Pose2D   leftPos = new Pose2D(DistanceUnit.MM, 0, lateralDistance / 2, AngleUnit.RADIANS, 0),
                    rightPos = new Pose2D(DistanceUnit.MM, 0, -lateralDistance / 2, AngleUnit.RADIANS, 0),
                    perpPos = new Pose2D(DistanceUnit.MM, perpendicularDistance, 0, AngleUnit.RADIANS, 0);
    public double ticksPerRevolution = 2000;


    private DcMotorEx left, right, perpendicular;
    private double[] lastEncoderValues, currentEncoderValues;
    public ThreeDeadWheelLocalizer(HardwareMap hm){
        //TODO: put the right names here
        left = hm.get(DcMotorEx.class, "eM3");
        right = hm.get(DcMotorEx.class, "cM3");
        perpendicular = hm.get(DcMotorEx.class, "cM0");
        currentPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        lastPosition = currentPosition;
        lastEncoderValues = new double[] {0, 0, 0};
    }

    public double getRealDistanceFromTicks(double ticks){
        return ticks / ticksPerRevolution * Math.PI * wheelDiameter;
    }
    public static double getDistance(Pose2D x, Pose2D y){
        double ax = x.getX(DistanceUnit.MM), ay = x.getY(DistanceUnit.MM), bx = y.getX(DistanceUnit.MM), by = y.getY(DistanceUnit.MM);
        return Math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
    }

    public static double NormalizeAngle(double angleInRadians){
        while (angleInRadians < 0)
            angleInRadians += Math.PI * 2;

        while(angleInRadians > Math.PI * 2)
            angleInRadians -= Math.PI * 2;

        return angleInRadians;
    }

    @Override
    public synchronized void update() {
        currentEncoderValues = new double[] {getRealDistanceFromTicks(left.getCurrentPosition()), getRealDistanceFromTicks(right.getCurrentPosition()), getRealDistanceFromTicks(perpendicular.getCurrentPosition())};
        double H = currentPosition.getHeading(AngleUnit.RADIANS), X = currentPosition.getX(DistanceUnit.MM), Y = currentPosition.getY(DistanceUnit.MM);
        double dh = (currentEncoderValues[0] - lastEncoderValues[0]) / lateralDistance - (currentEncoderValues[1] - lastEncoderValues[1]) / lateralDistance;
        H += dh;

        H = NormalizeAngle(H);

        double dF = ((currentEncoderValues[1] - lastEncoderValues[1]) +
                (currentEncoderValues[0] - lastEncoderValues[0])) / 2;
        double dS = (currentEncoderValues[2] - lastEncoderValues[2]) - perpendicularDistance * dh;

        double R0 = dF / H, R1 = dS / H;

        double relX = R0 * Math.sin(dh) - R1 * (1 - Math.cos(dh));
        double relY = R1 * Math.sin(dh) + R0 * (1 - Math.cos(dh));


        X += relX * Math.cos(H) - relY * Math.sin(H);
        Y += relY * Math.cos(H) + relX * Math.sin(H);

        currentPosition = new Pose2D(DistanceUnit.MM, X, Y, AngleUnit.RADIANS, H);

        lastEncoderValues = currentEncoderValues;

        FtcDashboard.getInstance().getTelemetry().addData("X", currentPosition.getX(DistanceUnit.MM));
        FtcDashboard.getInstance().getTelemetry().addData("Y", currentPosition.getY(DistanceUnit.MM));
        FtcDashboard.getInstance().getTelemetry().addData("heading", currentPosition.getHeading(AngleUnit.DEGREES));
        FtcDashboard.getInstance().getTelemetry().addData("dF", dF);
        FtcDashboard.getInstance().getTelemetry().addData("dS", dS);
        FtcDashboard.getInstance().getTelemetry().addData("relX", relX);
        FtcDashboard.getInstance().getTelemetry().addData("relY", relY);
        FtcDashboard.getInstance().getTelemetry().addData("dh", dh);

       /* TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("green")
                .fillCircle(currentPosition.getX(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH), 5)
                .setFill("red")
                .strokeLine(currentPosition.getX(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH),
                        (5 + currentPosition.getX(DistanceUnit.INCH)) * Math.sin(H), (currentPosition.getY(DistanceUnit.INCH) + 5) * Math.cos(H));

        FtcDashboard.getInstance().sendTelemetryPacket(packet);*/
    }

    @Override
    public synchronized void beaconUpdate(Pose2D pose) {
        currentPosition = pose;
    }
}
