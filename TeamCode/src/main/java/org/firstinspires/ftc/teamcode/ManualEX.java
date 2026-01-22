package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Ramp;

@TeleOp(name = "manualEX")
public class ManualEX extends OpMode {

    GoBildaPinpointDriver ODM;
    FieldOdometry odom;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake, R = Ramp
    MotorEx motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;
    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max, volts, distance, maxV, maxD;
    AnalogInput laser;
    MecanumDrive mec;
    GamepadEx driver, operator;
    Launcher launcherS;
    Intake intakeS;
    Ramp rampS;
    Drive driveS;
    double lP;

    @Override
    public void init() {
        motorFL = new MotorEx(hardwareMap,"motorFL",Motor.GoBILDA.RPM_312); //EH0
        motorFR = new MotorEx(hardwareMap,"motorFR",Motor.GoBILDA.RPM_312); //CH0
        motorBL = new MotorEx(hardwareMap,"motorBL",Motor.GoBILDA.RPM_312); //EH1
        motorBR = new MotorEx(hardwareMap,"motorBR",Motor.GoBILDA.RPM_312); //CH1
        motorLR = new MotorEx(hardwareMap,"motorLR"); //CH2
        motorLL = new MotorEx(hardwareMap,"motorLL"); //EH2
        motorI = new MotorEx(hardwareMap,"motorI",Motor.GoBILDA.RPM_223); //CH3
        motorR = new MotorEx(hardwareMap,"motorR",Motor.GoBILDA.RPM_312); //EH3

//        odom = new FieldOdometry(hardwareMap,"ODM");

        laser = hardwareMap.get(AnalogInput.class, "LIDAR");

        launcherS = new Launcher(hardwareMap);
        intakeS = new Intake(hardwareMap);
        rampS = new Ramp(hardwareMap);
        driveS = new Drive(hardwareMap,telemetry);

        motorFL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorI.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        ODM.setOffsets(60,170, DistanceUnit.MM);
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        ODM.resetPosAndIMU();
        Pose2D startingPos = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
        ODM.setPosition(startingPos);

        maxV = laser.getMaxVoltage();
        maxD = 1000.0;

        mec = new MecanumDrive(motorFL,motorFR,motorBL,motorBR);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        lP = 0;
    }

    @Override
    public void loop() {

        ODM.update();
//        odom.update();
        volts = laser.getVoltage();
        distance = (volts/maxV)*maxD;

        //sets the direction that each wheel will spin
        motorBR.setInverted(true);
        motorBL.setInverted(true);
        motorFL.setInverted(true);

        axial = driver.getLeftY(); //gets input of up and down of left stick for the forward and backwards robot driving
        lateral = driver.getLeftX(); //gets input of left and right of left stick for the robot strafing
        yaw = driver.getRightX(); //gets input of left and right of right stick for the robot turning

        driver.readButtons();
        operator.readButtons();


        //Odometry auto for field centric driving
        Pose2D pos = ODM.getPosition();
        double heading = ODM.getHeading(AngleUnit.DEGREES);

//        driveS.fieldCentricDrive(lateral,axial,yaw);
        mec.driveFieldCentric(lateral, axial, yaw, heading);

        if (gamepad1.dpad_right && lP <= 1) {
            lP += 0.01;
        } else if (gamepad1.dpad_left && lP > 0) {
            lP -= 0.01;
        }

//        Pose2D OP = odom.getFieldPose();

        telemetry.addData("XPos (Inch): ",pos.getX(DistanceUnit.INCH));
        telemetry.addData("YPos (Inch): ",pos.getY(DistanceUnit.INCH));
        telemetry.addData("Heading: ",heading);
        telemetry.addData("Launcher Status: ",launcherS.getStatus());
        telemetry.addData("Intake Status: ",intakeS.getStatus());
        telemetry.addData("Ramp Status: ",rampS.getStatus());
        telemetry.update();

        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            launcherS.enable(0.4);
        } else if (operator.wasJustPressed(GamepadKeys.Button.X) || driver.wasJustPressed(GamepadKeys.Button.X)) {
            launcherS.enable(0.5);
        } else if (operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3 || driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
            launcherS.enable(1);
        } else if (operator.wasJustPressed(GamepadKeys.Button.Y) || driver.wasJustPressed(GamepadKeys.Button.Y)) {
            launcherS.disable();
        }

        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeS.forward();
        } else if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            intakeS.reverse();
        } else if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            intakeS.disable();
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP) || driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            rampS.forward();
        } else if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            rampS.reverse();
        } else if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            rampS.disable();
        }

        //Strafe left. FL and BR spin backwards; FR and BL spin forwards
        //Strafe right. FR and BL spin backwards; FL and BR spin forwards
    }

}
