package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;

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

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        ODM.setOffsets(0,0, DistanceUnit.INCH);
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ODM.resetPosAndIMU();
        Pose2D startingPos = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
        ODM.setPosition(startingPos);

    }

    @Override
    public void loop() {

        ODM.update();

        //sets the direction that each wheel will spin
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
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

        powerFL = -forward + strafe + yaw;
        powerFR = -forward + strafe - yaw;
        powerBL = forward + strafe + yaw;
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
        telemetry.addData("Frequency: ",frequency);
        telemetry.update();

        /*
       //show the input amount from each stick on driver hub
        telemetry.addData("Driving Input: ",axial);
        telemetry.addData("Strafing Input: ",lateral);
        telemetry.addData("Turning Input: ",yaw);

        //show the power of each wheel as a percentage on driver hub
        telemetry.addData("Front-Left Wheel Power: ",powerFL);
        telemetry.addData("Front-Right Wheel Power: ",powerFR);
        telemetry.addData("Back-Left Wheel Power: ",powerBL);
        telemetry.addData("Back-Right Wheel Power: ",powerBR);
         */

        if (gamepad2.right_bumper || gamepad1.right_bumper) { //Turn on launcher
            motorLL.setPower(-0.40);
            motorLR.setPower(0.40);
        } else { //Turn off launcher
            motorLL.setPower(0);
            motorLR.setPower(0);
        }

        if (gamepad2.left_bumper || gamepad1.left_bumper) {
            motorI.setPower(1);
            motorR.setPower(0.6);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            motorI.setPower(-1);
            motorR.setPower(-0.6);
        } else {
            motorI.setPower(0);
            motorR.setPower(0);
        }


        //Strafe left. FL and BR spin backwards; FR and BL spin forwards
        //Strafe right. FR and BL spin backwards; FL and BR spin forwards
    }

}
