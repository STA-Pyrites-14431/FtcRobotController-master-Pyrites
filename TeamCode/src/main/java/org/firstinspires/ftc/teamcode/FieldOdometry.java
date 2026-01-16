package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldOdometry {
    private GoBildaPinpointDriver pinpoint;
    private double originX = 0;
    private double originY = 0;
    private double originHeading = 0;

    public FieldOdometry (HardwareMap hwMap, String deviceName) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, deviceName);
        pinpoint.update();
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }
    public void update() {
        pinpoint.update();
    }
    public Pose2D getFieldPose() {
        double rawX = pinpoint.getPosX(DistanceUnit.INCH);
        double rawY = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        double fieldHeading = heading+originHeading;

        double cos = Math.cos(fieldHeading);
        double sin = Math.sin(fieldHeading);

        double fieldX = (rawX*cos-rawY*sin) + originX;
        double fieldY = (rawX*sin+rawY*cos) + originY;

        return new Pose2D(DistanceUnit.INCH,fieldX,fieldY,AngleUnit.DEGREES,fieldHeading);
    }
    public double getY() {
        return getFieldPose().getY(DistanceUnit.INCH);
    }
    public double getX() {
        return getFieldPose().getX(DistanceUnit.INCH);
    }
    public double getHeading() {
        return getFieldPose().getHeading(AngleUnit.DEGREES);
    }

    public void resetOrigintoCurrent() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();

        originX = p.getX(DistanceUnit.INCH);
        originY = p.getY(DistanceUnit.INCH);
        originHeading = p.getHeading(AngleUnit.DEGREES);
    }

    public void setFieldOrigin(Pose2D start) {
        originX = start.getX(DistanceUnit.INCH);
        originY = start.getY(DistanceUnit.INCH);
        originHeading = start.getHeading(AngleUnit.DEGREES);
    }
}
