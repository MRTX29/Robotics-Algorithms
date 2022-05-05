
function sysCall_init()

    --Defining objects
    left_wheel = sim.getObjectHandle('Pioneer_p3dx_leftMotor')
    right_wheel = sim.getObjectHandle('Pioneer_p3dx_rightMotor')
    robot_pose = sim.getObjectHandle('robot_pose')
    ref_point = sim.getObjectHandle('ref_point')
    
    --Parameters of the Robot
    pose = updateRobotPose()
    wheel_radius = 0.195/2
    b = 0.1655
    vref = 0.25
    e = 0.24
    k = 1

end


function sysCall_actuation()
    local wL, wR
    local ptraj, vtraj, poff, voff
     
    
    --The following instruction is needed since 4.1.0 version
    simHandlePath(sim.handle_all_except_explicit, sim.getSimulationTimeStep())
    
    --Calculate trajectory point (ptraj and vtraj)
        ptraj, vtraj = getTrajectoryPoint()
    
    --Calulate OffCenterPoint of the trajectory (poff, voff)
        poff, voff = getOffCenterPoint(ptraj, vtraj, e)

    -- Calculate wheels speed (wL, wR)
        wL, wR = kinematicControl(poff, voff, pose, k)
    
    -- Set wheels motors
        sim.setJointTargetVelocity(left_wheel, wL)
        sim.setJointTargetVelocity(right_wheel, wR)
    
end


function sysCall_sensing()

    --Update robot Pose
        pose = updateRobotPose()
end

function sysCall_cleanup()

end

function getTrajectoryPoint()
    local position, orientation
    local linear_vel, angular_vel
    local ptraj, vtraj
    
    position = sim.getObjectPosition(ref_point, -1)
    orientation = sim.getObjectOrientation(ref_point, -1)
    linear_vel, angular_vel = sim.getObjectVelocity(ref_point)
    
    if(orientation[3] > 0) then
        ptraj = {position[1], position[2], orientation[2] - math.pi/2}
    else
        ptraj = {position[1], position[2], math.pi/2 - orientation[2]}
    end
    
        vtraj = {linear_vel[1], linear_vel[2], angular_vel[3]}
        return ptraj, vtraj
end

function getOffCenterPoint(ptraj, vtraj, e)
    local xc, yc, vxc, vyc
    
    xc = ptraj[1] + e * math.cos(ptraj[3])
    yc = ptraj[2] + e * math.sin(ptraj[3])
    
    vxc = vtraj[1] - e * vtraj[3] * math.sin(ptraj[3])
    vyc = vtraj[2] + e * vtraj[3] * math.cos(ptraj[3])
    
    return {xc,yc},{vxc,vyc}
end

function kinematicControl(ptraj, vtraj, pose, k)
    local vxc, vyc, ex, ey, uwL, uwR
    
    -- Calculate OffCenterPoint of pose robot
        xc = pose[1] + e * math.cos(pose[3])
        yc = pose[2] + e * math.sin(pose[3])
        
    -- Calculate error in X and Y (ex, ey)
        ex =  ptraj[1] - xc 
        ey =  ptraj[2] - yc

        
    -- Calculate vxc, vyc
        vxc = vtraj[1] + k * ex 
        vyc = vtraj[2] + k * ey
    
    -- Desired speed OffCenterPoint with PV or PD (vxc and vyc)
            uwL = (1/wheel_radius) * (((math.cos(pose[3]) + (b/e) * math.sin(pose[3])) * vxc) + ((math.sin(pose[3]) - (b/e) * math.cos(pose[3])) * vyc))
            uwR = (1/wheel_radius) * (((math.cos(pose[3]) - (b/e) * math.sin(pose[3])) * vxc) + ((math.sin(pose[3]) + (b/e) * math.cos(pose[3])) * vyc))
        
    return uwL, uwR
end

-- CHECKED
function updateRobotPose()
    local pose
    position = sim.getObjectPosition(robot_pose, -1)
    orientation = sim.getObjectOrientation(robot_pose, -1)
    pose = {position[1], position[2], orientation[3]}
    return pose
end


