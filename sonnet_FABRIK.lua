-- Vector3 utility functions
local Vector3 = {}
Vector3.__index = Vector3

function Vector3.new(x, y, z)
    return setmetatable({x = x or 0, y = y or 0, z = z or 0}, Vector3)
end

function Vector3:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
end

function Vector3:normalize()
    local len = self:length()
    if len > 0 then
        return Vector3.new(self.x / len, self.y / len, self.z / len)
    end
    return Vector3.new(0, 0, 0)
end

function Vector3:dot(other)
    return self.x * other.x + self.y * other.y + self.z * other.z
end

function Vector3:cross(other)
    return Vector3.new(
        self.y * other.z - self.z * other.y,
        self.z * other.x - self.x * other.z,
        self.x * other.y - self.y * other.x
    )
end

function Vector3.__add(a, b)
    return Vector3.new(a.x + b.x, a.y + b.y, a.z + b.z)
end

function Vector3.__sub(a, b)
    return Vector3.new(a.x - b.x, a.y - b.y, a.z - b.z)
end

function Vector3.__mul(a, b)
    if type(a) == "number" then
        return Vector3.new(a * b.x, a * b.y, a * b.z)
    elseif type(b) == "number" then
        return Vector3.new(a.x * b, a.y * b, a.z * b)
    end
end

function Vector3:clone()
    return Vector3.new(self.x, self.y, self.z)
end

function Vector3:distance(other)
    return (self - other):length()
end

-- Matrix3 for rotations
local Matrix3 = {}
Matrix3.__index = Matrix3

function Matrix3.new(m11, m12, m13, m21, m22, m23, m31, m32, m33)
    return setmetatable({
        m11 = m11 or 1, m12 = m12 or 0, m13 = m13 or 0,
        m21 = m21 or 0, m22 = m22 or 1, m23 = m23 or 0,
        m31 = m31 or 0, m32 = m32 or 0, m33 = m33 or 1
    }, Matrix3)
end

function Matrix3.identity()
    return Matrix3.new(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    )
end

function Matrix3.fromAxisAngle(axis, angle)
    local x, y, z = axis.x, axis.y, axis.z
    local c = math.cos(angle)
    local s = math.sin(angle)
    local t = 1 - c
    
    return Matrix3.new(
        t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
        t*x*y + s*z, t*y*y + c,   t*y*z - s*x,
        t*x*z - s*y, t*y*z + s*x, t*z*z + c
    )
end

function Matrix3:multiplyVector(v)
    return Vector3.new(
        self.m11 * v.x + self.m12 * v.y + self.m13 * v.z,
        self.m21 * v.x + self.m22 * v.y + self.m23 * v.z,
        self.m31 * v.x + self.m32 * v.y + self.m33 * v.z
    )
end

function Matrix3:multiply(other)
    return Matrix3.new(
        self.m11 * other.m11 + self.m12 * other.m21 + self.m13 * other.m31,
        self.m11 * other.m12 + self.m12 * other.m22 + self.m13 * other.m32,
        self.m11 * other.m13 + self.m12 * other.m23 + self.m13 * other.m33,
        
        self.m21 * other.m11 + self.m22 * other.m21 + self.m23 * other.m31,
        self.m21 * other.m12 + self.m22 * other.m22 + self.m23 * other.m32,
        self.m21 * other.m13 + self.m22 * other.m23 + self.m23 * other.m33,
        
        self.m31 * other.m11 + self.m32 * other.m21 + self.m33 * other.m31,
        self.m31 * other.m12 + self.m32 * other.m22 + self.m33 * other.m32,
        self.m31 * other.m13 + self.m32 * other.m23 + self.m33 * other.m33
    )
end

function Matrix3:transpose()
    return Matrix3.new(
        self.m11, self.m21, self.m31,
        self.m12, self.m22, self.m32,
        self.m13, self.m23, self.m33
    )
end

function Matrix3:inverse()
    local det = self.m11 * (self.m22 * self.m33 - self.m23 * self.m32) -
                self.m12 * (self.m21 * self.m33 - self.m23 * self.m31) +
                self.m13 * (self.m21 * self.m32 - self.m22 * self.m31)
    
    if math.abs(det) < 1e-6 then
        return Matrix3.identity() -- Return identity if matrix is singular
    end
    
    local invDet = 1 / det
    
    return Matrix3.new(
        (self.m22 * self.m33 - self.m23 * self.m32) * invDet,
        (self.m13 * self.m32 - self.m12 * self.m33) * invDet,
        (self.m12 * self.m23 - self.m13 * self.m22) * invDet,
        
        (self.m23 * self.m31 - self.m21 * self.m33) * invDet,
        (self.m11 * self.m33 - self.m13 * self.m31) * invDet,
        (self.m13 * self.m21 - self.m11 * self.m23) * invDet,
        
        (self.m21 * self.m32 - self.m22 * self.m31) * invDet,
        (self.m12 * self.m31 - self.m11 * self.m32) * invDet,
        (self.m11 * self.m22 - self.m12 * self.m21) * invDet
    )
end

function Matrix3:clone()
    return Matrix3.new(
        self.m11, self.m12, self.m13,
        self.m21, self.m22, self.m23,
        self.m31, self.m32, self.m33
    )
end

-- Quaternion utilities (for rotation representation and operations)
local Quaternion = {}
Quaternion.__index = Quaternion

function Quaternion.new(x, y, z, w)
    return setmetatable({x = x or 0, y = y or 0, z = z or 0, w = w or 1}, Quaternion)
end

function Quaternion.fromAxisAngle(axis, angle)
    local halfAngle = angle * 0.5
    local s = math.sin(halfAngle)
    return Quaternion.new(
        axis.x * s,
        axis.y * s,
        axis.z * s,
        math.cos(halfAngle)
    )
end

function Quaternion:toMatrix3()
    local x, y, z, w = self.x, self.y, self.z, self.w
    local xx, xy, xz = x*x, x*y, x*z
    local yy, yz, zz = y*y, y*z, z*z
    local wx, wy, wz = w*x, w*y, w*z
    
    return Matrix3.new(
        1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy),
        2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx),
        2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)
    )
end

function Quaternion:normalize()
    local len = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
    if len > 0 then
        return Quaternion.new(self.x / len, self.y / len, self.z / len, self.w / len)
    end
    return Quaternion.new(0, 0, 0, 1)
end

function Quaternion:multiply(other)
    return Quaternion.new(
        self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
        self.w * other.y + self.y * other.w + self.z * other.x - self.x * other.z,
        self.w * other.z + self.z * other.w + self.x * other.y - self.y * other.x,
        self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
    )
end

-- Joint class
local Joint = {}
Joint.__index = Joint

function Joint.new(position, axisOfRotation, minAngle, maxAngle)
    return setmetatable({
        position = position:clone(),
        axisOfRotation = axisOfRotation:normalize(),
        minAngle = minAngle or -math.pi, -- Default to -180 degrees
        maxAngle = maxAngle or math.pi,  -- Default to 180 degrees
        rotation = Matrix3.identity(),    -- Rotation relative to parent
        worldRotation = Matrix3.identity() -- Global orientation
    }, Joint)
end

-- Extract rotation angle around axis from a direction vector
function Joint:getAngleAroundAxis(targetDir, baseDir)
    -- Project vectors onto plane perpendicular to rotation axis
    local axisNorm = self.axisOfRotation:normalize()
    
    -- Remove axis-aligned component from vectors
    local baseProj = baseDir - axisNorm * baseDir:dot(axisNorm)
    local targetProj = targetDir - axisNorm * targetDir:dot(axisNorm)
    
    -- Normalize the projected vectors
    baseProj = baseProj:normalize()
    targetProj = targetProj:normalize()
    
    -- Calculate angle between the projected vectors
    local cosAngle = math.min(1, math.max(-1, baseProj:dot(targetProj)))
    local angle = math.acos(cosAngle)
    
    -- Determine sign of angle using cross product
    local cross = baseProj:cross(targetProj)
    if cross:dot(axisNorm) < 0 then
        angle = -angle
    end
    
    return angle
end

-- Apply angle constraints to a rotation
function Joint:applyConstraints(targetDirection, referenceDirection)
    -- First, ensure we're only rotating around the specified axis
    -- by projecting the motion onto the plane perpendicular to the rotation axis
    local axisNorm = self.axisOfRotation:normalize()
    
    -- Extract the component of targetDirection that's perpendicular to the rotation axis
    local axisComponent = axisNorm * targetDirection:dot(axisNorm)
    local targetInPlane = targetDirection - axisComponent
    
    -- Extract the component of referenceDirection that's perpendicular to the rotation axis
    local refAxisComponent = axisNorm * referenceDirection:dot(axisNorm)
    local refInPlane = referenceDirection - refAxisComponent
    
    -- If the projected vectors are too small, fall back to reference
    if targetInPlane:length() < 1e-6 then
        -- The target is aligned with rotation axis, use a perpendicular vector to reference
        local perpAxis = Vector3.new(1, 0, 0)
        if math.abs(perpAxis:dot(axisNorm)) > 0.9 then
            perpAxis = Vector3.new(0, 1, 0)
        end
        targetInPlane = perpAxis - axisNorm * perpAxis:dot(axisNorm)
    end
    
    if refInPlane:length() < 1e-6 then
        -- The reference is aligned with rotation axis, create perpendicular vector
        local perpAxis = Vector3.new(1, 0, 0)
        if math.abs(perpAxis:dot(axisNorm)) > 0.9 then
            perpAxis = Vector3.new(0, 1, 0)
        end
        refInPlane = perpAxis - axisNorm * perpAxis:dot(axisNorm)
    end
    
    -- Normalize the projected vectors
    targetInPlane = targetInPlane:normalize()
    refInPlane = refInPlane:normalize()
    
    -- Calculate angle between the projected vectors
    local cosAngle = math.min(1, math.max(-1, refInPlane:dot(targetInPlane)))
    local angle = math.acos(cosAngle)
    
    -- Determine sign of angle using cross product
    local cross = refInPlane:cross(targetInPlane)
    if cross:dot(axisNorm) < 0 then
        angle = -angle
    end
    
    -- Clamp angle to constraints
    local clampedAngle = math.max(self.minAngle, math.min(self.maxAngle, angle))
    
    -- Create rotation matrix for the clamped angle around the rotation axis
    local rotMatrix = Matrix3.fromAxisAngle(axisNorm, clampedAngle)
    
    -- Apply rotation to reference direction to get constrained direction
    -- Preserve the original length of the target direction
    local constrainedDir = rotMatrix:multiplyVector(refInPlane)
    local targetLength = targetDirection:length()
    
    -- Preserve any component along the rotation axis to maintain joint structure
    return (constrainedDir * targetLength) + axisComponent
end

-- FABRIK Solver
local FABRIK = {}
FABRIK.__index = FABRIK

function FABRIK.new()
    return setmetatable({
        joints = {},
        boneLengths = {},
        totalLength = 0,
        tolerance = 0.01,
        maxIterations = 10
    }, FABRIK)
end

function FABRIK:addJoint(joint)
    table.insert(self.joints, joint)
    
    -- Calculate bone lengths
    if #self.joints > 1 then
        local prevJoint = self.joints[#self.joints - 1]
        local length = joint.position:distance(prevJoint.position)
        table.insert(self.boneLengths, length)
        self.totalLength = self.totalLength + length
    end
    
    return self
end

function FABRIK:updateWorldRotations()
    -- Update world rotations based on parent relationships
    local worldRot = Matrix3.identity()
    
    for i, joint in ipairs(self.joints) do
        -- Update world rotation by combining with parent
        joint.worldRotation = worldRot:multiply(joint.rotation)
        worldRot = joint.worldRotation:clone()
    end
end

function FABRIK:solve(targetPosition, iterations)
    local numJoints = #self.joints
    if numJoints < 2 then return false end
    
    iterations = iterations or self.maxIterations
    
    -- Create a copy of joint positions for calculation
    local positions = {}
    for i, joint in ipairs(self.joints) do
        positions[i] = joint.position:clone()
    end
    
    -- Store initial bone directions for reference planes
    local initialDirections = {}
    for i = 1, numJoints - 1 do
        local dir = (self.joints[i+1].position - self.joints[i].position):normalize()
        initialDirections[i] = dir:clone()
    end
    
    -- Check if target is reachable
    local rootPos = positions[1]
    local distanceToTarget = rootPos:distance(targetPosition)
    
    if distanceToTarget > self.totalLength then
        -- Target unreachable - just extend chain in best direction
        local direction = (targetPosition - rootPos):normalize()
        
        -- Recompute positions in forward direction with strict plane constraints
        positions[1] = rootPos:clone() -- Ensure root is fixed
        
        for i = 1, numJoints - 1 do
            local joint = self.joints[i]
            local axisNorm = joint.axisOfRotation:normalize()
            
            -- Apply constraints: rotate around axis within min/max angle
            local refDir = initialDirections[i]
            
            -- For the constrained direction, we need to:
            -- 1. Work entirely in the plane perpendicular to rotation axis
            -- 2. Apply angle limits
            
            -- Project target direction onto the rotation plane
            local targetVec = (targetPosition - positions[i]):normalize()
            local axisComponent = axisNorm * targetVec:dot(axisNorm)
            local targetInPlane = (targetVec - axisComponent):normalize()
            
            -- Project reference direction onto the rotation plane
            local refAxisComponent = axisNorm * refDir:dot(axisNorm)
            local refInPlane = (refDir - refAxisComponent):normalize()
            
            -- Calculate angle between projected vectors
            local cosAngle = math.min(1, math.max(-1, refInPlane:dot(targetInPlane)))
            local angle = math.acos(cosAngle)
            
            -- Determine rotation sign
            local cross = refInPlane:cross(targetInPlane)
            if cross:dot(axisNorm) < 0 then
                angle = -angle
            end
            
            -- Clamp to constraints
            angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
            
            -- Apply rotation in the plane perpendicular to axis
            local rotMatrix = Matrix3.fromAxisAngle(axisNorm, angle)
            local rotatedDir = rotMatrix:multiplyVector(refDir)
            
            -- Position next joint using the strictly constrained direction
            positions[i+1] = positions[i] + rotatedDir * self.boneLengths[i]
            
            -- Update rotation matrix
            joint.rotation = rotMatrix:clone()
        end
    else
        -- Target is reachable, use FABRIK iteration with strict plane constraints
        local iter = 0
        local distToTarget = positions[numJoints]:distance(targetPosition)
        
        while distToTarget > self.tolerance and iter < iterations do
            -- BACKWARD PASS: Move end effector to target and work backward
            positions[numJoints] = targetPosition:clone()
            
            for i = numJoints - 1, 1, -1 do
                -- Standard FABRIK backward pass
                local direction = (positions[i] - positions[i+1]):normalize()
                positions[i] = positions[i+1] + direction * self.boneLengths[i]
            end
            
            -- FORWARD PASS: Fix the root and work forward with strict constraints
            positions[1] = self.joints[1].position:clone() -- Reset root
            
            for i = 1, numJoints - 1 do
                local joint = self.joints[i]
                local axisNorm = joint.axisOfRotation:normalize()
                
                -- Get reference direction (from initial pose)
                local refDir = initialDirections[i]
                
                -- Project the backward-pass direction onto the rotation plane
                local targetDir = (positions[i+1] - positions[i]):normalize()
                
                -- Extract the component along rotation axis (this should be preserved)
                local refDotAxis = refDir:dot(axisNorm)
                
                -- Project reference and target directions to plane perpendicular to axis
                local refInPlane = (refDir - axisNorm * refDotAxis):normalize()
                local targetInPlane = (targetDir - axisNorm * targetDir:dot(axisNorm)):normalize()
                
                -- Calculate angle between projected vectors
                local cosAngle = math.min(1, math.max(-1, refInPlane:dot(targetInPlane)))
                local angle = math.acos(cosAngle)
                
                -- Determine rotation sign
                local cross = refInPlane:cross(targetInPlane)
                if cross:dot(axisNorm) < 0 then
                    angle = -angle
                end
                
                -- Clamp to constraints
                angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
                
                -- Create rotation that keeps us in the correct plane
                local rotMatrix = Matrix3.fromAxisAngle(axisNorm, angle)
                
                -- Calculate constrained direction that:
                -- 1. Preserves the component along rotation axis
                -- 2. Only rotates the perpendicular component
                local constrainedDir = rotMatrix:multiplyVector(refDir)
                
                -- Position the next joint using the constrained direction
                positions[i+1] = positions[i] + constrainedDir * self.boneLengths[i]
                
                -- Update rotation matrix
                joint.rotation = rotMatrix:clone()
            end
            
            -- Check if we're close enough to target
            distToTarget = positions[numJoints]:distance(targetPosition)
            iter = iter + 1
        end
    end
    
    -- Update joint positions and orientations
    for i, joint in ipairs(self.joints) do
        joint.position = positions[i]:clone()
    end
    
    -- Update global rotations
    self:updateWorldRotations()
    
    return true
end

-- Example usage
local function createExample()
    -- Create example IK chain with joint constraints
    local solver = FABRIK.new()
    
    -- Root joint (Z-axis rotation) - only allows rotation in XY plane
    local root = Joint.new(
        Vector3.new(0, 0, 0),
        Vector3.new(0, 0, 1),  -- Z-axis rotation
        -math.pi/4,            -- -45 degrees
        math.pi/4              -- 45 degrees
    )
    
    -- Middle joint (Y-axis rotation) - only allows rotation in XZ plane
    local middle = Joint.new(
        Vector3.new(1, 0, 0),
        Vector3.new(0, 1, 0),  -- Y-axis rotation
        -math.pi/3,            -- -60 degrees
        math.pi/3              -- 60 degrees
    )
    
    -- End joint (Z-axis rotation) - only allows rotation in XY plane
    local end_joint = Joint.new(
        Vector3.new(2, 0, 0),
        Vector3.new(0, 0, 1),  -- Z-axis rotation
        -math.pi/2,            -- -90 degrees
        math.pi/2              -- 90 degrees
    )
    
    -- Add joints to solver
    solver:addJoint(root)
    solver:addJoint(middle)
    solver:addJoint(end_joint)
    
    return solver
end

-- Demonstration function
local function demonstrateFABRIK()
    local solver = createExample()
    
    -- Target position
    local target = Vector3.new(1.5, 1.0, 0.5)
    
    -- Print initial positions
    print("Initial positions:")
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d: (%.2f, %.2f, %.2f)", 
            i, joint.position.x, joint.position.y, joint.position.z))
        if i < #solver.joints then
            print(string.format("  Rotation axis: (%.2f, %.2f, %.2f)",
                joint.axisOfRotation.x, joint.axisOfRotation.y, joint.axisOfRotation.z))
        end
    end
    
    -- Store initial bone directions
    local initialDirs = {}
    for i = 1, #solver.joints - 1 do
        local joint = solver.joints[i]
        local nextJoint = solver.joints[i+1]
        initialDirs[i] = (nextJoint.position - joint.position):normalize()
    end
    
    -- Solve IK
    solver:solve(target)
    
    -- Print new positions
    print("\nAfter solving for target " .. 
          string.format("(%.2f, %.2f, %.2f):", target.x, target.y, target.z))
    
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d: (%.2f, %.2f, %.2f)", 
            i, joint.position.x, joint.position.y, joint.position.z))
    end
    
    -- Print constrained angles
    print("\nJoint angles relative to their rotation axes:")
    for i = 1, #solver.joints - 1 do
        local joint = solver.joints[i]
        local nextJoint = solver.joints[i+1]
        
        -- Calculate angle between initial and current directions in the plane
        -- perpendicular to rotation axis
        local axisNorm = joint.axisOfRotation:normalize()
        local initialDir = initialDirs[i]
        local currentDir = (nextJoint.position - joint.position):normalize()
        
        -- Project both directions onto plane perpendicular to axis
        local initialInPlane = (initialDir - axisNorm * initialDir:dot(axisNorm)):normalize()
        local currentInPlane = (currentDir - axisNorm * currentDir:dot(axisNorm)):normalize()
        
        -- Calculate angle between projections
        local cosAngle = math.min(1, math.max(-1, initialInPlane:dot(currentInPlane)))
        local angle = math.acos(cosAngle)
        
        -- Determine sign
        local cross = initialInPlane:cross(currentInPlane)
        if cross:dot(axisNorm) < 0 then
            angle = -angle
        end
        
        print(string.format("Joint %d angle: %.2f degrees (min: %.2f, max: %.2f)", 
            i, math.deg(angle), math.deg(joint.minAngle), math.deg(joint.maxAngle)))
    end
    
    -- Check if target is reachable
    local endEffector = solver.joints[#solver.joints].position
    local distToTarget = endEffector:distance(target)
    print(string.format("\nDistance from end effector to target: %.6f", distToTarget))
end

-- Run demonstration
demonstrateFABRIK()