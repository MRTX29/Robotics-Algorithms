

function sysCall_init()

    --Initialization of the articulations
    left_arm=sim.getObjectHandle('Left_lego_arm_joint')
    right_arm=sim.getObjectHandle('Right_lego_arm_joint')
    left_wheel=sim.getObjectHandle('left_motor')
    right_wheel=sim.getObjectHandle('right_motor')
    
    --Maximum distance of the sensors
    max_distance_lateral = 1.2
    max_distance_sonar = 0.5
    min_distance_sonar = 0.08
    
    --Initialization of the sensors
    right_front_sensor = sim.getObjectHandle('ir_front_right')
    right_back_sensor = sim.getObjectHandle('ir_rear_right')
    left_front_sensor = sim.getObjectHandle('ir_front_left')
    left_back_sensor = sim.getObjectHandle('ir_rear_left')
    sonar = sim.getObjectHandle('sonar_sensor')
    ir_sensor = sim.getObjectHandle('tcrt5000_sensor')
    
    --Parameters
    wheel_radius = 0.03
    max_speed = 0.15
    b = 0.05
    a = 0.037
    e = 0.24
    k = 0.01
    
    --Default sensors distance
    distanceLeft1 = max_distance_lateral
    distanceLeft2 = max_distance_lateral
    distanceRight1 = max_distance_lateral
    distanceRight2 = max_distance_lateral
    distanceSonar = max_distance_sonar
    
    lastTime = sim.getSimulationTime()  --Current Time of the simulation
    States = {"Follow wall","Turn slow","Make curve","Stop"} 
    state = 1 -- Initial state
    direction = 1 -- Follow right 1, follow left -1
    speed_reduction = 0.25 -- speed reduction factor when turning
    cell_size = 0.5 -- cell size of the maze
    wallThick = 0.05 -- wall thickness of the maze
    sepUS = 0.0815 -- distance of US with respect to the robot wheels
    sepLIDAR = 0.041 
    black = 0.1 -- Line colour threshold for detecting black

    --Initialize the curve angular velocity and time
    curve_wL, curve_wR, minCurveTime = makeCurve(max_speed)

    --Initialize the turn angular velocity and time
    turn_wL, turn_wR, minTurnTime = turnSlow(max_speed)

    --Expected distance for the walls
    expected_front = cell_size/2 - sepUS - wallThick/2
    expected_lateral = cell_size/2 - sepLIDAR - wallThick/2
    
    print("Expected front: ", expected_front)
    print("Expected lateral: ", expected_lateral)
    
    end
    
    function sysCall_actuation()
    
        local wL, wR
        print("Actual state: " , state)
        
        if(state == 1) then     --Robot will follow the wall
            wL, wR = followWall(max_speed, distanceLeft1, distanceLeft2, distanceRight1, distanceRight2, expected_lateral) -- POTENTIAL MISTAKES!!!
            
        elseif(state == 2) then --Robot will turn slowly
            wL = turn_wL
            wR = turn_wR
        elseif(state == 3) then --Robot will make a curve
            wL = curve_wL
            wR = curve_wR
        else                    -- Robot will stop
            wL = 0
            wR = 0
        end
    
        sim.setJointTargetVelocity(left_wheel, wL)
        sim.setJointTargetVelocity(right_wheel, wR)
    end
    
    function sysCall_sensing()

        local distanceRight, line_colour

        --Distance and line sensors
        distanceRight1 = getDistance(right_front_sensor, max_distance_lateral)
        distanceRight2 = getDistance(right_back_sensor, max_distance_lateral)
        distanceLeft1 = getDistance(left_front_sensor, max_distance_lateral)
        distanceLeft2 = getDistance(left_back_sensor, max_distance_lateral)
        distanceSonar = getDistance(sonar, max_distance_sonar)
        line_colour = getLineColour(ir_sensor)

        distanceRight = (distanceRight1 + distanceRight2)/2
        
        --Check if the walls are detected
            wallFront = wallDetected(distanceSonar, expected_front,  7e-2)
            wallSide = wallDetected(distanceRight, expected_lateral,  3e-1)
            
            if(wallFront == true) then
            print("Wall front -  ", wallFront)
            end
            
            if(wallSide == true) then
            print("Wall side -  ", wallSide)
            end
            
        
        if(state == 1) then
            lastTime = sim.getSimulationTime()
            if wallFront then
                state = 2
            end
            if not wallSide then
                state = 3
            end
        elseif(state == 2) then
            local timeElapsed = ((sim.getSimulationTime() - lastTime) > minTurnTime)
            if wallSide and timeElapsed then
                state = 1
            end
        elseif(state == 3) then
            local timeElapsed = ((sim.getSimulationTime() - lastTime) > minCurveTime)
            if wallSide and timeElapsed then
                state = 1
            end
        end

        if(line_colour < 0.1)then
            state = 4
        end
        
    end
    
    function sysCall_cleanup()
    -- do some clean-up here
    end
    
    -- Return distance of the sensor or maximum distance if nothing was detected
    function getDistance(sensor, max_dist)
    
        local state, distance
        state, distance = sim.readProximitySensor(sensor)
    
        if(state < 1) then
            distance = max_dist
        end
    
        return distance
    end
    
    function getLineColour(sensor)
            
        value = sim.getVisionSensorImage(sensor + sim.handleflag_greyscale)
        return value[1]
    end

    --Function that makes Robot follow the wall in state 1 (The state variable cannot be change while simulation!)
    function followWall(v, dFL, dRL, dFR, dRR, dWallSide)
        
        local phi,d,alpha,wL,wR

        if(direction == 1) then
            phi = math.atan((dFR - dRR)/a)
            d = (0.5 * (dFR + dRR) - dWallSide)
        else
            phi = math.atan((dRL - dFL)/a)
            d = (dWallSide - 0.5 * (dFL + dRL))
        end

        alpha = phi + k * d

        wL = (v/wheel_radius) * (math.cos(alpha) + (b/e) * math.sin(alpha))
        wR = (v/wheel_radius) * (math.cos(alpha) - (b/e) * math.sin(alpha))

        return wL, wR

    end

    --It's the same as precompute curve in the video!
    function makeCurve(vref)
        local wref, wL, wR, t

        wref = (vref)/(cell_size/2)
        wL = (vref + b * direction * wref)/wheel_radius
        wR = (vref - b * direction * wref)/wheel_radius

         --Time needed to complete the curve
        t = (math.pi/2)/wref

        return wL, wR, t
    end


    --It's the same as precompute turn in the video!
    function turnSlow(vref)
        
        local wL, wR, t
        vturn = vref * speed_reduction
        -- Angular velocities calculated to turn the robot
        wL = -direction * vref/wheel_radius
        wR = direction * vref/wheel_radius

        --Time needed to complete the turn
        t = b * math.pi/2/(vref)
        
        return wL, wR, t
    end

    function wallDetected (distance, expectedDistance, tolerance)
        return math.abs(distance - expectedDistance) < tolerance 
    end
   
    
