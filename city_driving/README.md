# City Driving

## Requirement (expected)

- [ ] Wall Following
- [ ] Line Following
- [ ] Color Seqmentation (for carwash)
- [ ] Stop Sign Detector

## Nodes

### Line Follower

Outputs to: Interrupt

### Wall Follower

Outputs to Interrupt

### Interrupt

Outputs to: System Drive

3 modes:
- Normal Mode
    - Take in Wall Follower and Line Follower Drive messages
    - Only activate wall follower if detecting wall within certain distance
- Carwash Mode
    - "park" in front of car wash
    - drive until can't see blue anymore
    - turn on wall follower until see orange at certain distance
    - return to Normal
- Stopsign Mode
    - Triggered by stop sign at certain distance
    - Stop for some amount of time
    - return to Normal

### Stop Sign Detector

Outputs to: Stop Sign Homograpy

### Stop Sign Homography

Outputs to: Interrupt
