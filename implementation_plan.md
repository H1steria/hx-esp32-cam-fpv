# DHT11 Corruption Fix Implementation Plan

## Problem Statement
The DHT11 sensor readings are causing JPEG corruption in the air unit due to timing conflicts between I2C communication and camera operations.

## Current Issues
1. Air unit (I2C master) actively requests DHT11 data from auxiliary unit (I2C slave)
2. I2C communication interferes with camera data capture
3. DHT11 sensor reading requires precise timing that conflicts with camera operations

## Solution Overview
Implement a hybrid approach where:
1. Auxiliary unit continues as I2C slave for motor commands
2. Auxiliary unit pre-reads DHT11 data periodically and stores it
3. Air unit requests pre-read DHT11 data only when camera is idle
4. Minimize I2C transaction time to reduce interference

## Implementation Steps

### Phase 1: Auxiliary Unit Modifications
1. Add global variables to store latest DHT11 readings
2. Modify DHT11 task to store readings in global variables
3. Add new I2C command to send DHT11 data
4. Implement I2C handler for DHT11 data requests

### Phase 2: Air Unit Modifications
1. Modify DHT11 task to request data only when camera is idle
2. Implement timeout mechanisms to prevent blocking
3. Optimize I2C communication timing
4. Maintain existing motor command functionality

### Phase 3: Testing and Optimization
1. Verify DHT11 data transmission works correctly
2. Test for JPEG corruption elimination
3. Optimize timing parameters if needed
4. Validate all existing functionality remains intact

## Technical Details

### Auxiliary Unit Changes
- File: `esp32_aux/src/main.cpp`
- Add DHT11 data storage variables
- Modify DHT11 reading task to store data
- Add I2C command handler for DHT11 data requests
- Maintain existing motor control functionality

### Air Unit Changes
- File: `components/air/main.cpp`
- Modify DHT11 task to use optimized reading approach
- Add synchronization with camera operations
- Implement timeout handling
- Maintain existing I2C master functionality for motor commands

## Expected Benefits
1. Eliminate JPEG corruption caused by DHT11 readings
2. Maintain all existing functionality (motor control, etc.)
3. Continue DHT11 data reporting to ground station
4. Reduce I2C communication time and interference
5. Improve overall system reliability

## Risk Mitigation
1. Preserve existing code functionality with minimal changes
2. Implement timeout mechanisms to prevent blocking
3. Maintain synchronization with camera operations
4. Test thoroughly before deployment
