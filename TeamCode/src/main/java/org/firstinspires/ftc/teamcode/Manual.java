package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "manual")
public class Manual extends OpMode {

    GoBildaPinpointDriver ODM;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake, R = Ramp
    DcMotor motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;

    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max, volts, distance, maxV, maxD;
    AnalogInput laser;

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1
        motorLR = hardwareMap.get(DcMotor.class,"motorLR"); //CH2
        motorLL = hardwareMap.get(DcMotor.class,"motorLL"); //EH2
        motorI = hardwareMap.get(DcMotor.class,"motorI"); //CH3
        motorR = hardwareMap.get(DcMotor.class,"motorR"); //EH3
        laser = hardwareMap.get(AnalogInput.class, "LIDAR");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        ODM.setOffsets(0,0, DistanceUnit.INCH);
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ODM.resetPosAndIMU();
        Pose2D startingPos = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
        ODM.setPosition(startingPos);

        maxV = laser.getMaxVoltage();
        maxD = 1000.0;

    }

    @Override
    public void loop() {

        ODM.update();
        volts = laser.getVoltage();
        distance = (volts/maxV)*maxD;

        //sets the direction that each wheel will spin
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);


        axial = gamepad1.left_stick_y; //gets input of up and down of left stick for the forward and backwards robot driving
        lateral = gamepad1.left_stick_x; //gets input of left and right of left stick for the robot strafing
        yaw = gamepad1.right_stick_x; //gets input of left and right of right stick for the robot turning


        //Odometry auto for field centric driving
        Pose2D pos = ODM.getPosition();
        double heading = -ODM.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI/2)-heading);
        double sinAngle = Math.sin((Math.PI/2)-heading);

        double strafe = -axial * sinAngle + lateral * cosAngle;
        double forward = axial * cosAngle + lateral * sinAngle;

        double frequency = ODM.getFrequency();

        powerFL = forward + strafe + yaw;
        powerFR = -forward + strafe - yaw;
        powerBL = -forward + strafe + yaw;
        powerBR = forward + strafe - yaw;


        //sets the power for the wheels
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        telemetry.addData("XPos (Inch): ",pos.getX(DistanceUnit.INCH));
        telemetry.addData("YPos (Inch): ",pos.getY(DistanceUnit.INCH));
        telemetry.addData("Heading: ",Math.toDegrees(heading));
        telemetry.addData("Forward Speed: ",forward);
        telemetry.addData("Strafe Speed: ",strafe);
        telemetry.addData("Distance: ",distance);
        telemetry.addData("Voltage: ",laser.getVoltage());
        telemetry.update();

        if (gamepad2.right_bumper || gamepad1.right_bumper) { //Turn on launcher
            motorLL.setPower(0.45);
            motorLR.setPower(-0.45);
        } else if (gamepad1.y || gamepad2.y){ //Turn off launcher
            motorLL.setPower(1);
            motorLR.setPower(-1);
        } else if (gamepad1.x || gamepad2.x){
            motorLL.setPower(0.55);
            motorLR.setPower(-0.55);
        } else {
            motorLL.setPower(0);
            motorLR.setPower(0);
        }

        if (gamepad2.left_bumper || gamepad1.left_bumper) {
            motorI.setPower(1);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            motorI.setPower(-1);
        } else {
            motorI.setPower(0);
        }
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            motorR.setPower(0.5);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            motorR.setPower(-0.5);
        } else {
            motorR.setPower(0);
        }


        //Strafe left. FL and BR spin backwards; FR and BL spin forwards
        //Strafe right. FR and BL spin backwards; FL and BR spin forwards
    }

}
