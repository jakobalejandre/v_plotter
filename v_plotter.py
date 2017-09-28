# -*- coding: utf-8 -*-
"""
Created on Thu Feb 09 22:35:26 2017

@author: jakobalejandre

V-Plotter

TODO: Flip the x-axis so that data isn't backwards.
TODO: Redefine constants at top of script.
TODO: Bring assertions back.
TODO: Decompose long lines into several smaller ones (self-correcting).

"""

import time
import math
import numpy as np
import random
import os

###############################################################################
# CLI argument parsing / static declaration.
###############################################################################

# TODO: Consider which of these (if any) to pass through the CLI.
# Delay between steps, in seconds.
fltDelay = 0.01
# RADIUS of the gear, in meters.
fltGearRadius = 0.0075
# Minimum step size, in degrees.
fltMinStep = 1.8

# GPIO pins.
# Left pin GPIO numbers.
pinLeftA = 2
pinLeftB = 3
pinLeftC = 4
pinLeftD = 14
lsPinsLeft = [pinLeftA, pinLeftB, pinLeftC, pinLeftD]
# Right pin GPIO numbers.
pinRightA = 16
pinRightB = 20
pinRightC = 26
pinRightD = 21
lsPinsRight = [pinRightA, pinRightB, pinRightC, pinRightD]
# Dictionary of pins for ease of use in stepper functions.
dictMotorPins = {'left' : lsPinsLeft, 'right' : lsPinsRight}
# Pins for capacitive touch sensors.
pinCapBottom = 24
pinCapSide = 23

# Drawing surface coordinates.
fltOriginX = 0.0
fltOriginY = 0.0

# Distance between drawing raft connection point and center of drawing raft
# magnet.
fltRaftLength = 0.03886

# Number of steps that a keyboard press will move a stepper (only relevant
# to manual mode).
intKeyboardSteps = 10

# The distance (in meters) that the drawing raft moves while searching
# for the capacitive sensors.
fltTestDistance = 0.0025
# Time delay (in seconds) between test movements to find the capacitive
# sensors.
fltTestDelay = 1.0
# List of operations to get drawing point from capacitive sensors back to the
# drawing origin. 
lsOriginOperations = []
for i in range(0,8):
    tpOperation = (-1, 'left')
    lsOriginOperations.append(tpOperation)
for i in range(0,4):
    tpOperation = (1, 'right')
    lsOriginOperations.append(tpOperation)
# Used to get the operations with the correct number of steps (each operation
# listed previously is just repeated 10 times in the correct order).
lsOriginOperations = [i for i in lsOriginOperations
                      for j in xrange(intKeyboardSteps)]

# Account for raft length in starting position of drawing point.
fltOriginY += fltRaftLength

# Drawing point tracking.
fltXCurrent = fltOriginX
fltYCurrent = fltOriginY
# Counters to track the pin state.
intCurrentLeftPin = 0
intCurrentRightPin = 0
# Dictionary of pin states.
dictPins = {0:[1,0,1,0], 1:[0,1,1,0], 2:[0,1,0,1], 3:[1,0,0,1]}

# Size specifications (all in meters).
# X distance from left motor to left side of drawing surface.
fltLeftMotorX = 0.044515
# X distance from left side of drawing surface to right motor.
fltRightMotorX = 0.305275
# X length of drawing surface (width).
fltDrawingX = 0.26165
# Y length of drawing surface (height).
fltDrawingY = 0.1758
# Vertical length from bottom of drawing surface to motor.
fltMotorY = 0.20633
# Fraction on the Y surface that can be drawn on (used to enforce margins).
fltDrawingFractionY = 1.0
# Distance between the motors.
# fltMotorDistance = fltLeftMotorX + fltRightMotorX
fltMotorDistance = 0.34979

# Some size sanity checks:
# TODO: Some buffer has to be added in.
#assert (fltRightMotorX - fltLeftMotorX) == fltDrawingX
assert (fltMotorY > fltDrawingY)
assert fltRightMotorX > 0.0
assert fltDrawingX > 0.0
assert fltDrawingY > 0.0

# TODO: These will need to be changed, depending on testing.
fltYMax = 100.0
fltYMin = 0.0
fltXMin = 1.0
fltXMax = 24.0

# Some fake data, for testing.
# TODO: Change this back to random.randint(0,100) once things have been
# debugged.
lsTemperature = [random.randint(40,70) for i in range(1,25)]
lsHour = range(1,25)

# Running this in simulation mode (all motor movement is turned off).
boolSim = os.name == 'nt'
if not boolSim:
    import RPi.GPIO as GPIO
# Used for tracking the position of desired coordinates, as well as the 
# position for actual coordinates.
lsDesiredX = []
lsDesiredY = []
lsActualX = []
lsActualY = []
lsActualX.append(fltXCurrent)
lsActualY.append(fltYCurrent)
# TODO: Remove after testing. Used to keep track of step operations that break
# the recursive shuffling function.
lsDesiredOperations = []

#------------------------------------------------------------------------------
# CLI argument parsing / static declaration.
#------------------------------------------------------------------------------

###############################################################################
# Conversion functions.
###############################################################################

# General mapping.
#******************************************************************************

# Add (filled in) points to a list.
def fillInPoints(lsOriginal, intIterations = 1):
    """ Add (filled in) points to a list.
    lsOriginal : list - coordinates of the data (mapped or unmapped).
    intIterations : int - Number of times that this process of filling 
        intermediates should be repeated. Defaults to 1.
    Returns lsFilled, a list of the coordinates of the data, with intermediate
        points added. Assumes a linear change between all coordinates.
    TODO: Calculate the length of the returned list.
    """
    assert len(lsOriginal) > 1
    
    # Loop through number of intermediate fillings.
    for j in range(0,intIterations):
        # New (filled) list.
        lsFilled = []
        for i in range(0, len(lsOriginal) - 1):
            lsFilled.append(lsOriginal[i])
            fltIntermediate = (lsOriginal[i] +
                               (lsOriginal[i + 1] - lsOriginal[i]) / 2.0)
            lsFilled.append(fltIntermediate)
            # Edge case to add last element of original list.
            if i == (len(lsOriginal) - 2):
                lsFilled.append(lsOriginal[i + 1])
        lsOriginal = lsFilled
            
    # Return the filled list.
    return lsFilled  

# Map the desired plotting data to the drawing surface.
def mapToDrawingSurface(lsX, lsY):
    """ Map the desired plotting data to the drawing surface.
    lsX : list - X coordinates of the data.
    lsY : list - Y coordinates of the data.
    Returns lsMappedX, the X coordinates of the data mapped to fit on the 
        drawing surface.
    Returns lsMappedY, the Y coordinates of the data mapped to fit on the
        drawing surface.
    """
    assert len(lsX) == len(lsY)
    
    # Map the input lists to (0,1)
    lsPercentX = [((fltX - fltXMin) / (fltXMax - fltXMin)) for fltX in lsX]
    lsPercentY = [((fltY - fltYMin) / (fltYMax - fltYMin)) for fltY in lsY]
                  
    # Map the X percentages to drawing surface coordinates.
    lsMappedX = [fltX * fltDrawingX for fltX in lsPercentX]
    # Map the Y percentages to drawing surface coordinates. Slightly more
    # complicated because of the Y margins.
    fltInitY = ((1 - fltDrawingFractionY) / 2.0)
    lsMappedY = [fltInitY + (fltY *
                             fltDrawingFractionY *
                             fltDrawingY) for fltY in lsPercentY]
    
    # All Y points are shifted up by the length of the drawing raft.
    lsMappedY = [fltPointY + fltRaftLength for fltPointY in lsMappedY]
    
    # Increasing the number of intermediate points.
    # TODO: Bring back if testing does not work.
    lsMappedX = fillInPoints(lsMappedX, 1)
    lsMappedY = fillInPoints(lsMappedY, 1)
    
    # Adding the intermediate steps for the first vertical movement.
    # TODO: Bring back if testing does not work.
    lsStartX = fillInPoints([fltOriginX, fltOriginX], 1)[1:]
    lsStartY = fillInPoints([fltOriginY, lsMappedY[0]], 1)[1:]
    
    # Prepending the starting coordinates to the mapped x and y lists.
    # TODO: Bring back if testing does not work.
    lsMappedX = lsStartX + lsMappedX
    lsMappedY = lsStartY + lsMappedY
                             
    return (lsMappedX, lsMappedY)
    
#******************************************************************************
# General mapping.

# Coordinates -> steps.
#******************************************************************************

# Shuffles motor operations as evenly as possible.
def shuffleOperations(lsLeft, lsRight):
    """ Shuffles motor operations as evenly as possible.
    lsLeft : list - a list of zeros, denoting left stepper operations.
    lsRight : list - a list of ones, denoting right stepper operations.
    Returns lsOperations, a list of left and right motor operations to perform.
        The ordering of the operations is the key component of this function.
    Note : Calling functions take care of the direction of the motor operation.
    """
    assert lsLeft > 0, 'Left motor has zero operations.'
    assert lsRight > 0, 'Right motor has zero operations.'
    
    # Base case - one or both operation lists contain one operation.
    ###########################################################################
    
    if len(lsLeft) == 1 or len(lsRight) == 1:
        
        # Case where both operation lists contain one operation.
        if len(lsLeft) == 1 and len(lsRight) == 1:
            return [0,1]
        
        # Determine which of the lists is larger/smaller.
        if len(lsLeft) > len(lsRight):
            intSmaller = lsRight[0]
            lsLarger = lsLeft[:]
        else:
            intSmaller = lsLeft[0]
            lsLarger = lsRight[:]
            
        # The smaller element is placed at the center (or as close as possible)
        # of the larger list.
        intIndex = len(lsLarger) / 2
        lsLarger.insert(intIndex, intSmaller)
        return lsLarger
    
    # Main operations - make both sets of operations even and divide by two.
    ###########################################################################
    
    # Making both sets of operations even.
    if len(lsLeft) % 2 == 1:
        intRemainderLeft = 1
        lsLeft = lsLeft[1:]
    else:
        intRemainderLeft = 0
    if len(lsRight) % 2 == 1:
        intRemainderRight = 1
        lsRight = lsRight[1:]
    else:
        intRemainderRight = 0
        
    lsRemainderLeft = [0] * intRemainderLeft
    lsRemainderRight = [1] * intRemainderRight
        
    # Integer indices that split the lists in half.
    intHalfLeft = len(lsLeft) / 2
    intHalfRight = len(lsRight) / 2
        
    # Recursive calls to function with half of input list. Remaining elements
    # get placed in the middle.
    return (shuffleOperations(lsLeft[:intHalfLeft], lsRight[:intHalfRight]) + 
            lsRemainderLeft + lsRemainderRight +
            shuffleOperations(lsLeft[intHalfLeft:], lsRight[intHalfRight:]))
    
# Convert string distance to stepper angle change.
def convertDistanceToAngle(fltDistance, strLeftRight):
    """ Convert string distance to stepper angle change.
    fltDistance : float - desired distance change in string.
    strLeftRight : string - left or right string described by fltDistance. 
        Possible options are 'left' and 'right'.
    Returns fltTheta : float - the angle change required for a change of
        fltDistance in the string.
        
    Convention: A positive (+) fltDistance is an increase in the length of the
        string. A negative (-) fltDistance is a decrease in the length of the
        string (it is being retracted). A positive fltTheta is a CW rotation
        of the stepper. A negative fltTheta is a CCW rotation of the stepper.
    """
    if (strLeftRight not in ['left', 'right']):
        raise ValueError("strLeftRight must be either 'left' or 'right'." +
                         " Value passed in was " + str(strLeftRight) + ".")
    
    if (strLeftRight == 'left'):
        # Left stepper.
        intThetaDirection = np.sign(fltDistance)
    else:
        # Right stepper.
        intThetaDirection = -1 * np.sign(fltDistance)
        
    fltTheta = (abs(fltDistance) * 180) / (math.pi * fltGearRadius)
    fltTheta = fltTheta * intThetaDirection
    return fltTheta
    
# Convert stepper angle change to discrete number of steps.
def convertAngleToSteps(fltTheta):
    """Convert stepper angle change to discrete number of steps.
    fltTheta : float - Desired angle change.
    Returns intSteps - The discrete number of steps required to achieve the 
    desired fltTheta. Always returns a positive number (theta sign is ignored).
    """
    # TODO: Might have to be careful with rounding here to make sure that 
    # the number of steps isn't exceeding the drawing surface.
    return int(round(fltTheta / fltMinStep))
    
# Convert a cartesian coordinate (x,y) to a left and right string length.
def convertCartToStringLength(fltX, fltY):
    """ Convert a cartesian coordinate (x,y) to a left and right string length.
    fltX : float - X coordinate.
    fltY : float - Y coordinate.
    Returns (fltLeftStringLength, fltRightStringLength), a tuple of the 
        lengths of the left and right strings (in meters) needed to have the 
        drawing point at (x,y). Both fltLeftStringLength and
        fltRightStringLength should be strictly positive.
    """
    fltLeftStringLength = math.sqrt((fltX + fltLeftMotorX)**2 +
                                      (fltMotorY - fltY)**2)
    fltRightStringLength = math.sqrt((fltRightMotorX - fltX)**2 +
                                       (fltMotorY - fltY)**2)
    
    return (fltLeftStringLength, fltRightStringLength)
    
# Converts a set of L/R steps to a list of stepper operations.
def convertStepsToPath(intStepsLeft, intStepsRight):
    """ Converts a set of L/R steps to a list of stepper operations.
    intStepsLeft : int - the number of steps that the left motor has to take.
    intStepsRight : int - the number of steps that the right motor has to take.
    Returns lsOperations, a list of left and right motor operations to 
        perform. An operation is a tuple of steps and a string 
        ('left'/'right') representing the left or right motor. A step in
        lsOperations must be an integer and != 0.
    Note : A postive step is a CW rotation. A negative step is a CCW rotation.
    """
    assert type(intStepsLeft) == int, 'The number of steps must be an integer'
    assert type(intStepsRight) == int, 'The number of steps must be an integer'
    
    # Convert steps into lists that can be shuffled. Left steps are a list
    # of 0s. Right steps are a list of 1s. Sign is added at a later point.
    lsStepsLeft = [0 for i in range(0, abs(intStepsLeft))]
    lsStepsRight = [1 for i in range(0, abs(intStepsRight))]
    lsDesiredOperations.append([intStepsLeft, intStepsRight])
    if intStepsLeft == 0 or intStepsRight == 0:
        lsStepsShuffled = []
    else:
        lsStepsShuffled = shuffleOperations(lsStepsLeft, lsStepsRight)
    
    # Sign of left and right steps (determines the direction).
    intSignLeft = np.sign(intStepsLeft)
    intSignRight = np.sign(intStepsRight)
    
    lsOperations = []
    
    # More information added to the shuffled steps.
    for intStep in lsStepsShuffled:
        if intStep == 0:
            # Left step.
            tpOperation = (intSignLeft, 'left')
        else:
            # Right step.
            tpOperation = (intSignRight, 'right')
            
        lsOperations.append(tpOperation)
    
    return lsOperations
   
# Calculate a path required to move between sets of coordinates.
def calculatePath(fltXCurrent, fltYCurrent, fltXNew, fltYNew):
    """ Calculate a path required to move between sets of coordinates.
    fltXCurrent : float - the X coordinate of the drawing point before being
        moved.
    fltYCurrent : float - the Y coordinate of the drawing point before being
        moved.
    fltXNew : float - the X coordinate of the drawing point after being moved.
    fltYNew : float - the Y coordinate of the drawing point after being moved.
    Returns lsOperations, a list of the operations required to move from the 
        original coordinates to the new coordinates.
    Note that this function does not return the new set of coordinates, because
        this is tracked by the stepper functions.
    """
    # TODO: Make sure it's fine to have this buffer.
    # TODO: Bring these back after testing.
    # Assertions to make sure the drawing point stays within the drawing
    # surface.
#    assert fltXCurrent >= 0.0
#    assert fltYCurrent >= 0.0
#    assert fltXCurrent <= (fltDrawingX * 1.01)
#    assert fltYCurrent <= (fltDrawingY * 1.01)
#    assert fltXNew >= 0.0
#    assert fltYNew >= 0.0
#    assert fltXNew <= fltDrawingX
#    assert fltYNew <= fltDrawingY

    # TODO: Make sure that this is correct.
    # Mirroring X-coordinate.
    # fltXCurrent = fltDrawingX - fltXCurrent
    # fltXNew = fltDrawingX - fltXNew
    
    # Converting the current set of coordinates to a (R/L) string length.
    (fltCurrLeftString,
     fltCurrRightString) = convertCartToStringLength(fltXCurrent, fltYCurrent)
    # Convert the new set of coordinates to a (R/L) string length.
    (fltNewLeftString,
     fltNewRightString) = convertCartToStringLength(fltXNew, fltYNew)
    
    # Calculate the length difference between the string sizes.
    fltDeltaLeft = fltNewLeftString - fltCurrLeftString
    fltDeltaRight = fltNewRightString - fltCurrRightString
    
    # Convert the length difference to a required theta.
    fltThetaLeft = convertDistanceToAngle(fltDeltaLeft, 'left')
    fltThetaRight = convertDistanceToAngle(fltDeltaRight, 'right')
    
    # Convert the required theta to a step quantity.
    # Positive steps are CW, negative steps are CCW.
    intStepsLeft = convertAngleToSteps(fltThetaLeft)
    intStepsRight = convertAngleToSteps(fltThetaRight)
    
    lsOperations = convertStepsToPath(intStepsLeft, intStepsRight)
    
    return lsOperations
    
#******************************************************************************
# Coordinates -> steps.

# Steps -> coordinates.
#******************************************************************************
    
# Converts a theta to a change in string length.
def convertAngleToDistance(fltTheta, strLeftRight):
    """ Converts a theta to a change in string length.
    fltTheta : float - the angle change of a stepper motor. A positive theta is
        a CW movement, a negative theta is a CCW movement.
    strLeftRight : string - the stepper that the change in theta applies to.
    Returns fltDistance : the change in length of a string. A positive value 
        means that the string is getting longer, while a negative value means
        that the string is being retracted.
    """
    if (strLeftRight not in ['left', 'right']):
        raise ValueError("strLeftRight must be either 'left' or 'right'." +
                         " Value passed in was " + str(strLeftRight) + ".")
        
    # Initializing with the proper sign.
    if (strLeftRight == 'left'):
        # Left - positive theta lengthens string.
        fltDistanceSign = 1.0
    else:
        # Right - positive theta retracts string.
        fltDistanceSign = -1.0
        
    # delta D = (pi * r * theta) / 180
    fltDistance = ((fltDistanceSign * math.pi * fltGearRadius * fltTheta) /
                   180)
    
    return fltDistance
    
# Converts two string lengths into an (x,y) coordinate.
def convertLengthToCart(fltLeftStringLength, fltRightStringLength):
    """ Converts two string lengths into an (x,y) coordinate.
    fltLeftStringLength : float - the length of the left string, in meters.
    fltRightStringLength : float - the length of the right string, in meters.
    Returns fltXNew, the updated X coordinate of the drawing point.
    Return fltYNew, the updated Y coordinate of the drawing point.
    Notes: This function uses some unfortunately messy trig. The basic premise
        is that the three sides of triangle, made by the two strings and the 
        (imaginary) line connecting the two motors, are all known distances.
        Using the Law of Cosines, we can solve for the angle between the 
        imaginary line and one of the strings, and then use basic sin/cos 
        identities to get the coordinates on the drawing surface.
    """
    assert fltLeftStringLength >= 0
    assert fltRightStringLength >= 0
    
    # Using the Law of Cosines to get the angle of between the left string
    # and the imaginary line connecting the two motors.
    fltAcosNumerator = (fltLeftStringLength ** 2 +
                        fltMotorDistance ** 2 -
                        fltRightStringLength ** 2)
    fltAcosDenominator = (2 * fltLeftStringLength * fltMotorDistance)
    fltAngleLeft = math.acos(fltAcosNumerator / 
                             fltAcosDenominator)
    
    # Now we use sin/cos properties to calculate the x/y distances from
    # the left motor.
    fltXFromLeft = math.cos(fltAngleLeft) * fltLeftStringLength
    fltYFromLeft = math.sin(fltAngleLeft) * fltLeftStringLength
    
    # Convert the distances from the left motor to drawing coordinates.
    # TODO: Think about if the abs() is correct or not.
    fltXNew = abs(fltLeftMotorX - fltXFromLeft)
    fltYNew = abs(fltMotorY - fltYFromLeft)
    
    return (fltXNew, fltYNew)

# Converts a step operation to a new set of drawing coordinates.
def convertStepsToCart(tpOperation, fltXCurrent, fltYCurrent):
    """ Converts a step operation to a new set of drawing coordinates.
    tpOperation : tuple - contains an instruction to turn CW (1) or CCW (-1),
        and a string ('left' or 'right') for the relevant motor to execute the
        operation.
    fltXCurrent : float - The current X coordinate of the drawing point.
    fltYCurrent : float - The current Y coordinate of the drawing point.
    Returns fltXNew, the updated X coordinate of the drawing point, after 
        making the desired step.
    Returns fltYNew, the updated Y coordinate of the drawing point, after
        making the desired step.
    """
    # TODO: Not sure if this is the correct operation.
    # fltXCurrent = fltDrawingX - fltXCurrent
    
    # Unpacking the operation tuple.
    intStep, strMotor = tpOperation
    
    # Convert the step to an angle (theta). A positive theta is a CW rotation,
    # while a negative is a CCW rotation.
    fltTheta = intStep * fltMinStep
    
    # Convert the theta into a change in the string length.
    fltDistance = convertAngleToDistance(fltTheta, strMotor)
    
    # Calculate the current string lengths.
    tpCurrentStringLengths = convertCartToStringLength(fltXCurrent,
                                                       fltYCurrent)
    
    fltStringCurrLeft, fltStringCurrRight = tpCurrentStringLengths
    
    # Calculate the new string lengths.
    if (strMotor == 'left'):
        fltStringNewLeft = fltStringCurrLeft + fltDistance
        fltStringNewRight = fltStringCurrRight
    else:
        fltStringNewLeft = fltStringCurrLeft
        fltStringNewRight = fltStringCurrRight + fltDistance
        
    # Calculate the new (x,y) coordinates.
    fltXNew, fltYNew = convertLengthToCart(fltStringNewLeft,
                                           fltStringNewRight)
    
    # TODO: Not sure if this is the correct operation either.
    # fltXNew = fltDrawingX - fltXNew
    
    return (fltXNew, fltYNew)

#******************************************************************************
# Steps -> coordinates.

#------------------------------------------------------------------------------
# Conversion functions.
#------------------------------------------------------------------------------

###############################################################################
# Stepper functions.
###############################################################################
    
# Change the pin state of a motor.
def changePinState(intStep, intCurrentPin, strMotor):
    """ Change the pin state of a motor.
    intStep : int - desired CW (1) or CCW (-1) movement.
    intCurrentPin : int - contains the current state of the (L/R) pin. The 
        correct side is passed in.
    strMotor : string - describes the motor that is being stepped.
    Returns lsNewPinState - a list of pin state information to move the 
        desired step.
    Returns intNewPin - the new state of the (L/R) pin.
    """
    assert intStep == 1 or intStep == -1
    
    # Properly increment/decrement the pin state.
    if strMotor == 'left':
        intNewPin = intCurrentPin - intStep
    else:
        intNewPin = intCurrentPin - intStep
    
    if intNewPin == -1:
        intNewPin = 3
    elif intNewPin == 4:
        intNewPin = 0
        
    lsNewPinState = dictPins[intNewPin]

    return (lsNewPinState, intNewPin)
        
        
# Execute a single tuple from a list of operations.
def moveStep(tpOperation, intCurrentLeftPin, intCurrentRightPin):
    """ Execute a single tuple from a list of operations.
    tpOperation : tuple - contains an instruction to turn CW (1) or CCW (-1),
        and a string ('left' or 'right') for the relevant motor to execute the
        operation.
    intCurrentLeftPin : int - the current state of the left motor pins.
    intCurrentRightPin : int - the current state of the right motor pins.
    Returns intNewLeftPin : int - the new state of the left motor pins.
    Returns intNewRightPin : int - the new state of the right motor pins.
    """
    assert len(tpOperation) == 2, 'The tuple length was greater than 2.'
    
    # Unpacking the operation tuple.
    intStep, strMotor = tpOperation
    
    # These are renamed for clarity. There is no guarantee that the returned
    # new pin will be different from the input current pin.
    intNewLeftPin, intNewRightPin = intCurrentLeftPin, intCurrentRightPin
    
    # Grab the new pin state.
    if strMotor == 'left':
        lsNewPinState, intNewLeftPin = changePinState(intStep,
                                                      intCurrentLeftPin,
                                                      strMotor)
    elif strMotor == 'right':
        lsNewPinState, intNewRightPin = changePinState(intStep,
                                                       intCurrentRightPin,
                                                       strMotor)
        
    # Loop through the new pin states, and make the changes for the relevant
    # motor.
    for intPinIndex, intPinState in enumerate(lsNewPinState):
        GPIO.output(dictMotorPins[strMotor][intPinIndex], intPinState)
        
    # Delay to make sure that the motor completes the step.
    time.sleep(fltDelay)
        
    return (intNewLeftPin, intNewRightPin)
        
#------------------------------------------------------------------------------
# Stepper functions.
#------------------------------------------------------------------------------

###############################################################################
# High-level plotting functions.
###############################################################################

# Verbose output to the console detailing movement.
def printMovement(fltXCurrent, fltYCurrent, fltXNew, fltYNew):
    """ Verbose output to the console detailing movement."""
    
    print("Moving from (" + str(fltXCurrent) + ", " +
          str(fltYCurrent) + ")")
    
    print("to" + " " * 10 + "(" + str(fltXNew) + ", " +
          str(fltYNew) + ").")
    
# Executes the movement(s) in an operation list.
def executeOperations(lsOperations, intCurrentLeftPin, intCurrentRightPin,
                      fltXCurrent, fltYCurrent):
    """ Execute the movement(s) in an operation list.
    lsOperations : list - operations that the stepper motors will do.
    intCurrentLeftPin : int - the current state of the left motor pins.
    intCurrentRightPin : int - the current state of the right motor pins.
    fltXCurrent : float - the current X coordinate of the drawing point.
    fltYCurrent : float - the current Y coordinate of the drawing point.
    Returns intCurrentLeftPin/intCurrentRightPin - the updated states of the
        left and right motor pins.
    Returns fltXCurrent, fltYCurrent - the new (x,y) coordinates of the 
        drawing point.
    """
    for tpOperation in lsOperations:
        if not boolSim:
            tpCurrentPins = moveStep(tpOperation, intCurrentLeftPin, 
                                     intCurrentRightPin)
            
            intCurrentLeftPin, intCurrentRightPin = tpCurrentPins
        
        # Calculate the new (x,y) coordinate caused by the step.
        fltXCurrent, fltYCurrent = convertStepsToCart(tpOperation,
                                                      fltXCurrent,
                                                      fltYCurrent)
        
        # Tracking the actual (x,y) coordinates.
        lsActualX.append(fltXCurrent)
        lsActualY.append(fltYCurrent)
        
    return (intCurrentLeftPin, intCurrentRightPin, fltXCurrent, fltYCurrent)
    
# Plot a specific (unmapped) data series.
def plotSeries(lsX, lsY, intCurrentLeftPin, intCurrentRightPin,
               fltXCurrent, fltYCurrent):
    """ Plot a specific (unmapped) data series.
    lsX : list - the unmapped X data point(s) to be plotted.
    lsY : list - the unmapped Y data point(s) to be plotted.
    intCurrentLeftPin : int - the current state of the left motor pins.
    intCurrentRightPin : int - the current state of the right motor pins.
    fltXCurrent : float - the current X coordinate of the drawing point.
    fltYCurrent : float - the current Y coordinate of the drawing point.
    Returns intCurrentLeftPin/intCurrentRightPin - the updated states of the
        left and right motor pins.
    Returns fltXCurrent, fltYCurrent - the new (x,y) coordinates of the 
        drawing point.
    """
    
    # Map data to drawing surface coordinates.
    lsMappedX, lsMappedY = mapToDrawingSurface(lsX, lsY)
    
    # Loop through the coordinates and plot them.
    for intCoordinateIndex in range(0,len(lsMappedX)):
        
        fltXNew = lsMappedX[intCoordinateIndex]
        fltYNew = lsMappedY[intCoordinateIndex]

        # Keeping track of the desired (x,y) coordinates.
        lsDesiredX.append(fltXNew)
        lsDesiredY.append(fltYNew)

        printMovement(fltXCurrent, fltYCurrent, fltXNew, fltYNew)
        
        # Calculate the operations required to move the drawing point.
        lsOperations = calculatePath(fltXCurrent, fltYCurrent, fltXNew,
                                     fltYNew)
        
        # Execute the operations.
        tpCurrentState = executeOperations(lsOperations, intCurrentLeftPin,
                                           intCurrentRightPin,
                                           fltXCurrent, fltYCurrent)
        
        (intCurrentLeftPin, intCurrentRightPin,
         fltXCurrent, fltYCurrent) = tpCurrentState
         
    return (intCurrentLeftPin, intCurrentRightPin, fltXCurrent, fltYCurrent)
    
# Return the plotting point to the origin.
def returnToOrigin(intCurrentLeftPin, intCurrentRightPin,
                   fltXCurrent, fltYCurrent):
    """ Return the plotting point to the origin.
    intCurrentLeftPin : int - the current state of the left motor pins.
    intCurrentRightPin : int - the current state of the right motor pins.
    fltXCurrent : float - the current X coordinate of the drawing point.
    fltYCurrent : float - the current Y coordinate of the drawing point.
    Returns intCurrentLeftPin/intCurrentRightPin - the updated states of the
        left and right motor pins.
    Returns fltXCurrent, fltYCurrent - the new (x,y) coordinates of the 
        drawing point.
    """
    # First, drawing point should move to bottom-right corner.
    # Then, it should move to origin (bottom-left).
    # lsX = [fltDrawingX, fltOriginX]
    # TODO: Remove this magic number.
    # TODO: Change back to 100 if testing does not work.
    intIntermediates = 5
    lsX = [fltDrawingX] * intIntermediates
    lsX.extend(list(np.linspace(fltDrawingX, fltOriginX, intIntermediates)))
    lsY = list(np.linspace(fltYCurrent, fltOriginY, intIntermediates))
    lsY.extend([fltOriginY] * intIntermediates)
    assert len(lsX) == len(lsY)

    print("Returning to origin.")

    for intCoordinateIndex in range(0,len(lsX)):
        
        fltXNew = lsX[intCoordinateIndex]
        fltYNew = lsY[intCoordinateIndex]

        # Keeping track of the desired (x,y) coordinates.
        lsDesiredX.append(fltXNew)
        lsDesiredY.append(fltYNew)

        printMovement(fltXCurrent, fltYCurrent, fltXNew, fltYNew)
        
        # Calculate the operations required to move the drawing point.
        lsOperations = calculatePath(fltXCurrent, fltYCurrent, fltXNew,
                                     fltYNew)
        
        # Execute the operations.
        tpCurrentState = executeOperations(lsOperations, intCurrentLeftPin,
                                           intCurrentRightPin,
                                           fltXCurrent, fltYCurrent)
        
        (intCurrentLeftPin, intCurrentRightPin,
         fltXCurrent, fltYCurrent) = tpCurrentState
         
    return (intCurrentLeftPin, intCurrentRightPin, fltXCurrent, fltYCurrent)

# Test movement of the plotter.    
def testMove(intCurrentLeftPin, intCurrentRightPin, fltXCurrent, fltYCurrent,
             fltXTestDistance, fltYTestDistance):
    """ Test movement of the plotter.
    TODO: Documentation.
    
    """
    fltXNew = fltXCurrent + fltXTestDistance
    fltYNew = fltYCurrent + fltYTestDistance
    
    printMovement(fltXCurrent, fltYCurrent, fltXNew, fltYNew)
    
    # Calculate the operations required to move the drawing point.
    lsOperations = calculatePath(fltXCurrent, fltYCurrent, fltXNew, fltYNew)
    
    # Execute the operations.
    tpCurrentState = executeOperations(lsOperations, intCurrentLeftPin,
                                       intCurrentRightPin,
                                       fltXCurrent, fltYCurrent)
    
    (intCurrentLeftPin, intCurrentRightPin,
     fltXCurrent, fltYCurrent) = tpCurrentState
     
    return (intCurrentLeftPin, intCurrentRightPin, fltXCurrent, fltYCurrent)

# Move the plotter from the capacitive sensors to the drawing origin.
def returnToDrawingOrigin(intCurrentLeftPin, intCurrentRightPin):
    # TODO: Write docstring for this function.
    
    print("Returning to drawing origin.")
    
    for tpOperation in lsOriginOperations:
        tpCurrentPins = moveStep(tpOperation, intCurrentLeftPin,
                                 intCurrentRightPin)
        
        intCurrentLeftPin, intCurrentRightPin = tpCurrentPins
        
    return (intCurrentLeftPin, intCurrentRightPin)

# Move the plotter until both of the capacitive sensors are hit.
def returnToCap(intCurrentLeftPin, intCurrentRightPin, fltXCurrent,
                fltYCurrent):
    """ Move the plotter until both of the capacitive sensors are hit.
    intCurrentLeftPin : int - the current state of the left motor pins.
    intCurrentRightPin : int - the current state of the right motor pins.
    fltXCurrent : float - the current X coordinate of the drawing point.
    fltYCurrent : float - the current Y coordinate of the drawing point.
    Returns intCurrentLeftPin/intCurrentRightPin - the updated states of the
        left and right motor pins.
    Returns fltXCurrent, fltYCurrent - the new (x,y) coordinates of the 
        drawing point.
    """
    
    # First attempt to find the side capacitive sensor.
    while not GPIO.input(pinCapSide):
        
        tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
                                  fltXCurrent, fltYCurrent,
                                  -1.0 * fltTestDistance, 0.0)
        
        (intCurrentLeftPin,
         intCurrentRightPin,
         fltXCurrent, 
         fltYCurrent) = tpCurrentState
         
        time.sleep(fltTestDelay)
        
    print("Found side capacitive sensor.")
         
    # Now back off the side by one test distance.
    tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
                              fltXCurrent, fltYCurrent,
                              fltTestDistance, 0.0)
    
    (intCurrentLeftPin,
     intCurrentRightPin,
     fltXCurrent, 
     fltYCurrent) = tpCurrentState
     
    time.sleep(fltTestDelay)
     
    # Now attempt to find the bottom capacitive sensor.
    while not GPIO.input(pinCapBottom):
        
        tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
                                  fltXCurrent, fltYCurrent,
                                  0.0, -1.0 * fltTestDistance)
        
        (intCurrentLeftPin,
         intCurrentRightPin,
         fltXCurrent, 
         fltYCurrent) = tpCurrentState
         
        time.sleep(fltTestDelay)
        
    print("Found bottom capacitive sensor.")
    
    # Back off the bottom by one test distance.
    tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
                              fltXCurrent, fltYCurrent,
                              0.0, fltTestDistance)
    
    (intCurrentLeftPin,
     intCurrentRightPin,
     fltXCurrent, 
     fltYCurrent) = tpCurrentState
       
    # Move from the capacitive origin to the drawing origin.
    returnToDrawingOrigin(intCurrentLeftPin, intCurrentRightPin)
         
    # Note that this returns the original drawing origin (should be 0, 0).
    return (intCurrentLeftPin, intCurrentRightPin, fltOriginX, fltOriginY)

#------------------------------------------------------------------------------
# High-level plotting functions.
#------------------------------------------------------------------------------

###############################################################################
# Keyboard-input movement.
###############################################################################

# Taken from:
# http://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-    ...
# detecting-keyboard-input-in-python-from-the-terminal

# See also:
# http://stackoverflow.com/questions/22397289/finding-the-values-of-the-    ...
# arrow-keys-in-python-why-are-they-triples

class _Getch:
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            try:
                self.impl = _GetchMacCarbon()
            except(AttributeError, ImportError):
                self.impl = _GetchUnix()

    def __call__(self): return self.impl()
    
class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()
        
class _GetchUnix:
    def __init__(self):
        import tty, sys, termios # import termios now or else you'll get the
        # Unix version on the Mac

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(3)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
class _GetchMacCarbon:
    """
    A function which returns the current ASCII key that is down;
    if no ASCII key is down, the null string is returned.  The
    page http://www.mactech.com/macintosh-c/chap02-1.html was
    very helpful in figuring out how to do this.
    """
    def __init__(self):
        import Carbon
        Carbon.Evt #see if it has this (in Unix, it doesn't)

    def __call__(self):
        import Carbon
        if Carbon.Evt.EventAvail(0x0008)[0]==0: # 0x0008 is the keyDownMask
            return ''
        else:
            #
            # The event contains the following info:
            # (what,msg,when,where,mod)=Carbon.Evt.GetNextEvent(0x0008)[1]
            #
            # The message (msg) contains the ASCII char which is
            # extracted with the 0x000000FF charCodeMask; this
            # number is converted to an ASCII character with chr() and
            # returned
            #
            (what,msg,when,where,mod)=Carbon.Evt.GetNextEvent(0x0008)[1]
            return chr(msg & 0x000000FF)
            
def getKey():
    inkey = _Getch()
    import sys
    for i in xrange(sys.maxint):
        k=inkey()
        if k<>'':break

    return k
    
# TODO: Work on documentation for this.
# Keyboard input of stepper operations.
def manualInput(intCurrentLeftPin, intCurrentRightPin, intSteps):
    """Keyboard input of stepper operations."""
    
    boolExit = True
    print "Keyboard control of stepper operations."
    print "Press any 3 (non-arrow) keys to escape."
    # Determines the desired stepper to move.
    strLeftRight = None
    print("No stepper currently selected. Press left or right arrow to " +
          "select a stepper.")
    
    while boolExit:
        strKey = getKey()
        if strKey == '\x1b[C':
            # Right.
            strLeftRight = 'right'
            print "Currently moving right motor."
            intDirection = 1
        elif strKey == '\x1b[D':
            # Left.
            strLeftRight = 'left'
            print "Currently moving left motor."
            intDirection = -1
        elif strKey == '\x1b[A':
            # Up.
            if strLeftRight is None:
                print "No stepper selected."
            else:
                tpOperation = (1 * intDirection, strLeftRight)
                # Allows the stepper to move several steps on a single key 
                # push.
                for i in range(0,intSteps):
                    
                    tpCurrentPins = moveStep(tpOperation, intCurrentLeftPin, 
                                     intCurrentRightPin)
            
                    intCurrentLeftPin, intCurrentRightPin = tpCurrentPins
        elif strKey == '\x1b[B':
            # Down.
            if strLeftRight is None:
                print "No stepper selected."
            else:
                tpOperation = (-1 * intDirection, strLeftRight)
                
                # Multiple steps per key press.
                for i in range(0,intSteps):
                    tpCurrentPins = moveStep(tpOperation, intCurrentLeftPin, 
                                         intCurrentRightPin)
                
                    intCurrentLeftPin, intCurrentRightPin = tpCurrentPins
        else: 
            # Escape sequence.
            boolExit = False
            print "Exiting keyboard input."
            
    return (intCurrentLeftPin, intCurrentRightPin)
        

#------------------------------------------------------------------------------
# Keyboard-input movement.
#------------------------------------------------------------------------------
        
###############################################################################
# Main execution.
###############################################################################

# TODO: Make fltXCurrent, fltYCurrent, intCurrentLeftPin, intCurrentRightPin
# all global so that code isn't crazy verbose and impossible to follow.
        
try:
    
    if not boolSim:
        # Refer to pins by GPIO numbers (Broadcom SOC channel).
        GPIO.setmode(GPIO.BCM)
        
        # Setting up all pins for output.
        for strMotor, lsPins in dictMotorPins.iteritems():
            print "Setting up pins for " + strMotor + " motor."
            for intPin in lsPins:
                GPIO.setup(intPin, GPIO.OUT)
                
        # Setting up the capacitive sensor pins for input.
        GPIO.setup(pinCapBottom, GPIO.IN)
        GPIO.setup(pinCapSide, GPIO.IN)
       
    # Keyboard-controlled movement of steppers.
    intCurrentLeftPin, intCurrentRightPin = manualInput(intCurrentLeftPin,
                                                        intCurrentRightPin,
                                                        intKeyboardSteps)
    
    # Testing out the ability to find the capacitive sensors.
    tpCurrentState = returnToCap(intCurrentLeftPin, intCurrentRightPin,
                                 fltXCurrent, fltYCurrent)
    
    (intCurrentLeftPin,
     intCurrentRightPin,
     fltXCurrent, 
     fltYCurrent) = tpCurrentState
     
    intCurrentLeftPin, intCurrentRightPin = manualInput(intCurrentLeftPin,
                                                        intCurrentRightPin,
                                                        intKeyboardSteps)
    
                
#    fltXTestDistance = 0.05
#    fltYTestDistance = 0.05
#    tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
#                              fltXCurrent, fltYCurrent, fltXTestDistance,
#                              fltYTestDistance)
#    
#    (intCurrentLeftPin,
#     intCurrentRightPin,
#     fltXCurrent, 
#     fltYCurrent) = tpCurrentState
#     
#    tpCurrentState = testMove(intCurrentLeftPin, intCurrentRightPin,
#                              fltXCurrent, fltYCurrent,
#                              -1.0 * fltXTestDistance, -1.0 * fltYTestDistance)
#    
#    (intCurrentLeftPin,
#     intCurrentRightPin,
#     fltXCurrent, 
#     fltYCurrent) = tpCurrentState
            
    # Plot some (test) time series data.
#    tpCurrentState = plotSeries(lsHour, lsTemperature, intCurrentLeftPin,
#                                intCurrentRightPin, fltXCurrent, fltYCurrent)
#    
#    # Unpack the current state.
#    (intCurrentLeftPin,
#     intCurrentRightPin,
#     fltXCurrent, 
#     fltYCurrent) = tpCurrentState
#     
#    # Bring the drawing point back to the origin.
#    tpCurrentState = returnToOrigin(intCurrentLeftPin, intCurrentRightPin,
#                                    fltXCurrent, fltYCurrent)
#    
#    # Unpack the current state.
#    (intCurrentLeftPin,
#     intCurrentRightPin,
#     fltXCurrent, 
#     fltYCurrent) = tpCurrentState
     
    # Plot some things (if this is a simulation).
    if boolSim:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(lsDesiredX, lsDesiredY, 'r-')
        plt.plot(lsActualX, lsActualY, 'k-')
        plt.xlim([-0.02, fltDrawingX])
        plt.ylim([-0.02, fltDrawingY])
        plt.grid()
        plt.legend(['Desired','Actual'])
        
except KeyboardInterrupt:

    print "Keyboard Interruption."
    
#except:
#    
#    print "Some other exception happened."

finally:
    if not boolSim:
        GPIO.cleanup()
    
#------------------------------------------------------------------------------
# Main execution.
#------------------------------------------------------------------------------