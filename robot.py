#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib.drive
import math
import wpimath
import phoenix5
import wpimath
from phoenix5 import WPI_TalonSRX
from phoenix5 import sensors
import phoenix5._ctre

#import rev
#import phoenix6
#from phoenix6 import hardware, controls, configs
#from phoenix6 import canbus
#import wpilib.CANTalon

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.m_leftDrive_Motor = WPI_TalonSRX(1)
        self.m_rightDrive_Motor = WPI_TalonSRX(0)
        self.m_rearDrive_Motor = WPI_TalonSRX(2)
        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()
        self.IMU = phoenix5.sensors.PigeonIMU(self.m_leftDrive_Motor)

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        #self.rightDrive.setInverted(True)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        #if self.timer.get() < 2.0:
        #    # Drive forwards half speed, make sure to turn input squaring off
        #    self.robotDrive.arcadeDrive(0.5, 0, squareInputs=False)
        #else:
        #    self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.IMU.setFusedHeading(0.0)

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        if self.controller.getRawButton(5):
            self.IMU.setFusedHeading(0.0)
            print(f"Reset Heading")
        self.m_drive()

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""

    def m_drive(self):

        x = -self.controller.getLeftX()
        y = -self.controller.getLeftY()
        z = self.controller.getRightX()
        
        x = self.apply_deadband(x, 0.05, 0.25)
        y = self.apply_deadband(y, 0.05, 0.25)
        z = self.apply_deadband(z, 0.05, 0.25)

        x = self.exponential_control(x, 2.5)
        y = self.exponential_control(y, 2.5)
        z = self.exponential_control(z, 2.5)

        heading = self.IMU.getFusedHeading()
        theta = heading * math.pi / 180
        yout = (x * math.sin(theta)) + (y * math.cos(theta))
        xout = (x * math.cos(theta)) - (y * math.sin(theta))
        y = yout
        x = xout

        right_wheel = -0.5 * x - math.sqrt(3) / 2 * y + z
        left_wheel = -0.5 * x + math.sqrt(3) / 2 * y + z
        rear_wheel = x + z

        self.m_leftDrive_Motor.set(left_wheel)
        self.m_rightDrive_Motor.set(right_wheel)
        self.m_rearDrive_Motor.set(rear_wheel)

    def exponential_control(self, input_val, exponent_val):
        return math.copysign(math.pow(abs(input_val), exponent_val), input_val) 

    def apply_deadband(self, value, deadband, maxMagnitude):
        magnitude = abs(value)
        if (magnitude > deadband):
            #if (maxMagnitude / deadband > 1.0E12):
                #if(value > 0.0):
                    #return value - deadband
                #else:
                    #return value + deadband    
            if (value > 0.0):
                return value * (maxMagnitude - deadband) + deadband
            else:
                return (value * (maxMagnitude - deadband) + deadband) * -1
        else:
            return 0.0


"""
    def m_drive(self):
        x = self.exponential_control(applyDeadband(
            self.controller.getLeftX() , 0.1, 0.25), 2.5)
        y = self.exponential_control(applyDeadband(
            self.controller.getLeftY() , 0.1, 0.25), 2.5)
        z = self.exponential_control(applyDeadband(
            self.controller.getRightX(), 0.1, 0.2), 2)

        heading = self.IMU.getFusedHeading()
        theta = heading * math.pi / 180
        yout = x * math.sin(theta) + y * math.cos(theta)
        xout = x * math.cos(theta) - y * math.sin(theta)
        y = yout
        x = xout

        right_wheel = -0.5 * x - math.sqrt(3) / 2 * y + z
        left_wheel = -0.5 * x + math.sqrt(3) / 2 * y + z
        rear_wheel = x + z

        self.m_leftDrive_Motor.set(left_wheel)
        self.m_rightDrive_Motor.set(right_wheel)
        self.m_rearDrive_Motor.set(rear_wheel)

    """



if __name__ == "__main__":
    wpilib.run(MyRobot)



