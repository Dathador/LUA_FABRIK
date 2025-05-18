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
        initialPosition = position:clone(),       -- Store the initial position
        localAxisOfRotation = axisOfRotation:normalize(),  -- Axis in local space
        worldAxisOfRotation = axisOfRotation:normalize(),  -- Axis in world space (will be updated)
        minAngle = minAngle or -math.pi, -- Default to -180 degrees
        maxAngle = maxAngle or math.pi,  -- Default to 180 degrees
        rotation = Matrix3.identity(),    -- Local rotation relative to parent
        worldRotation = Matrix3.identity(), -- Global orientation
        parent = nil,                     -- Reference to parent joint
        angle = 0                         -- Current rotation angle around axis
    }, Joint)
end

function Joint:setParent(parent)
    self.parent = parent
    return self
end

-- Update world axis orientation based on parent rotations
function Joint:updateWorldAxis()
    if self.parent then
        -- Transform local axis by parent's world rotation to get world axis
        self.worldAxisOfRotation = self.parent.worldRotation:multiplyVector(self.localAxisOfRotation)
    else
        -- Root joint - world axis is same as local axis
        self.worldAxisOfRotation = self.localAxisOfRotation:clone()
    end
end

-- Apply angle constraints to a rotation
function Joint:applyConstraints(targetDirection, referenceDirection)
    -- Use the world axis for constraints, not local axis
    local axisNorm = self.worldAxisOfRotation:normalize()
    
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
    
    -- Store the applied angle for later reference
    self.angle = clampedAngle
    
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
        maxIterations = 100
    }, FABRIK)
end

function FABRIK:addJoint(joint)
    local jointIndex = #self.joints + 1
    
    -- Set parent relationship
    if jointIndex > 1 then
        joint:setParent(self.joints[jointIndex-1])
        
        -- Calculate bone length
        local prevJoint = self.joints[jointIndex - 1]
        local length = joint.position:distance(prevJoint.position)
        table.insert(self.boneLengths, length)
        self.totalLength = self.totalLength + length
    end
    
    table.insert(self.joints, joint)
    return self
end

-- Calculate a basis for a joint based on its rotation axis
function Joint:calculateBasis()
    local up = self.worldAxisOfRotation:normalize()
    
    -- Find a perpendicular vector to 'up'
    local right = Vector3.new(1, 0, 0)
    if math.abs(up:dot(right)) > 0.9 then
        right = Vector3.new(0, 1, 0)
    end
    
    -- Make right perpendicular to up
    right = right - up * right:dot(up)
    right = right:normalize()
    
    -- Calculate forward as perpendicular to both up and right
    local forward = up:cross(right)
    
    return {
        right = right,
        up = up,
        forward = forward
    }
end

-- Update world rotations and axis orientations from root to tip
function FABRIK:updateWorldData()
    local worldRot = Matrix3.identity()
    
    for i, joint in ipairs(self.joints) do
        -- Update joint world rotation
        joint.worldRotation = worldRot:multiply(joint.rotation)
        
        -- Update the joint's world axis orientation
        joint:updateWorldAxis()
        
        -- Pass down rotations to children
        worldRot = joint.worldRotation:clone()
    end
end

-- Project a point onto the plane defined by a point and normal
local function projectPointOntoPlane(point, planePoint, planeNormal)
    local v = point - planePoint
    local dist = v:dot(planeNormal)
    return point - planeNormal * dist
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
    
    -- Update world data before solving to ensure correct constraints
    self:updateWorldData()
    
    -- Check if target is reachable
    local rootPos = positions[1]
    local distanceToTarget = rootPos:distance(targetPosition)
    
    -- Get the rotation planes for each joint
    local rotationPlanes = {}
    for i = 1, numJoints do
        local joint = self.joints[i]
        rotationPlanes[i] = {
            point = joint.position:clone(),
            normal = joint.worldAxisOfRotation:normalize()
        }
    end
    
    if distanceToTarget > self.totalLength then
        -- Target unreachable - just extend chain in best direction
        
        -- Recompute positions in forward direction with strict plane constraints
        positions[1] = rootPos:clone() -- Ensure root is fixed
        
        for i = 1, numJoints - 1 do
            local joint = self.joints[i]
            
            -- First joint: create rotation in the XY plane only (Z-axis rotation)
            if i == 1 then
                -- For first joint, directly calculate angle in XY plane
                local targetDir = targetPosition - positions[1]
                local targetXY = Vector3.new(targetDir.x, targetDir.y, 0):normalize()
                
                -- Calculate angle from X-axis
                local angle = math.atan(targetXY.y, targetXY.x)
                
                -- Clamp to joint limits
                angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
                joint.angle = angle
                
                -- Create rotation matrix for Z-axis rotation
                joint.rotation = Matrix3.fromAxisAngle(Vector3.new(0, 0, 1), angle)
                
                -- Update world rotations before continuing
                self:updateWorldData()
                
                -- Position next joint in XY plane
                local rotatedDir = Vector3.new(math.cos(angle), math.sin(angle), 0)
                positions[i+1] = positions[i] + rotatedDir * self.boneLengths[i]
            else
                -- For subsequent joints, rotate in the plane perpendicular to the joint's world axis
                local basis = joint:calculateBasis()
                
                -- Create a target vector in joint's local space
                local targetVec = (targetPosition - positions[i]):normalize()
                
                -- Project target onto rotation plane (perpendicular to world axis)
                local axisComponent = basis.up * targetVec:dot(basis.up)
                local targetInPlane = (targetVec - axisComponent):normalize()
                
                -- Get reference direction
                local refDir = initialDirections[i]
                local refInPlane = (refDir - basis.up * refDir:dot(basis.up)):normalize()
                
                -- Calculate angle between reference and target in plane
                local dotProduct = math.max(-1, math.min(1, refInPlane:dot(targetInPlane)))
                local angle = math.acos(dotProduct)
                
                -- Determine rotation direction
                local cross = refInPlane:cross(targetInPlane)
                if cross:dot(basis.up) < 0 then
                    angle = -angle
                end
                
                -- Clamp angle to joint limits
                angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
                joint.angle = angle
                
                -- Create rotation matrix around world axis
                joint.rotation = Matrix3.fromAxisAngle(basis.up, angle)
                
                -- Update world rotations for next joint
                self:updateWorldData()
                
                -- Calculate constrained direction
                local rotMatrix = Matrix3.fromAxisAngle(basis.up, angle)
                local constrainedDir = rotMatrix:multiplyVector(refInPlane)
                
                -- Position next joint
                positions[i+1] = positions[i] + constrainedDir * self.boneLengths[i]
            end
        end
    else
        -- Target is reachable, use FABRIK iteration with strict plane constraints
        local iter = 0
        local distToTarget = positions[numJoints]:distance(targetPosition)
        
        while distToTarget > self.tolerance and iter < iterations do
            -- BACKWARD PASS: Move end effector to target and work backward
            positions[numJoints] = targetPosition:clone()
            
            for i = numJoints - 1, 1, -1 do
                -- Direction from next joint to current
                local direction = (positions[i] - positions[i+1]):normalize()
                
                -- Standard FABRIK backward pass
                positions[i] = positions[i+1] + direction * self.boneLengths[i]
                
                -- If not root joint, project onto parent's rotation plane
                if i > 1 then
                    local parentJoint = self.joints[i-1]
                    local parentPlane = rotationPlanes[i-1]
                    
                    -- Ensure position stays on parent's rotation plane
                    positions[i] = projectPointOntoPlane(positions[i], parentPlane.point, parentPlane.normal)
                end
            end
            
            -- FORWARD PASS: Fix the root and work forward with strict constraints
            positions[1] = self.joints[1].position:clone() -- Reset root
            
            for i = 1, numJoints - 1 do
                local joint = self.joints[i]
                
                -- For first joint, only rotate around Z axis (XY plane)
                if i == 1 then
                    -- Calculate angle in XY plane
                    local direction = positions[i+1] - positions[i]
                    local angle = math.atan(direction.y, direction.x)
                    
                    -- Clamp to joint limits
                    angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
                    joint.angle = angle
                    
                    -- Create rotation matrix for Z-axis rotation
                    joint.rotation = Matrix3.fromAxisAngle(Vector3.new(0, 0, 1), angle)
                    
                    -- Position next joint in XY plane only
                    local rotatedDir = Vector3.new(math.cos(angle), math.sin(angle), 0)
                    positions[i+1] = positions[i] + rotatedDir * self.boneLengths[i]
                else
                    -- For subsequent joints, rotate in proper constraint plane
                    -- Update world data for accurate constraints
                    self:updateWorldData()
                    
                    -- Calculate the rotation plane for this joint
                    local basis = joint:calculateBasis()
                    
                    -- Get direction from backward pass
                    local backwardDir = positions[i+1] - positions[i]
                    
                    -- Project onto rotation plane to enforce constraints
                    local axisComponent = basis.up * backwardDir:dot(basis.up)
                    local dirInPlane = (backwardDir - axisComponent):normalize()
                    
                    -- Get reference direction
                    local refDir = initialDirections[i]
                    local refInPlane = (refDir - basis.up * refDir:dot(basis.up)):normalize()
                    
                    -- Calculate angle between reference and target in plane
                    local dotProduct = math.max(-1, math.min(1, refInPlane:dot(dirInPlane)))
                    local angle = math.acos(dotProduct)
                    
                    -- Determine rotation direction
                    local cross = refInPlane:cross(dirInPlane)
                    if cross:dot(basis.up) < 0 then
                        angle = -angle
                    end
                    
                    -- Clamp angle to joint limits
                    angle = math.max(joint.minAngle, math.min(joint.maxAngle, angle))
                    joint.angle = angle
                    
                    -- Create rotation matrix around world axis
                    joint.rotation = Matrix3.fromAxisAngle(basis.up, angle)
                    
                    -- Calculate constrained direction
                    local rotMatrix = Matrix3.fromAxisAngle(basis.up, angle)
                    local constrainedDir = rotMatrix:multiplyVector(refInPlane)
                    
                    -- Position next joint
                    positions[i+1] = positions[i] + constrainedDir * self.boneLengths[i]
                end
            end
            
            -- Check if we're close enough to target
            distToTarget = positions[numJoints]:distance(targetPosition)
            iter = iter + 1
        end
    end
    
    -- Update joint positions
    for i, joint in ipairs(self.joints) do
        joint.position = positions[i]:clone()
    end
    
    -- Final update of world rotations and axes
    self:updateWorldData()
    
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
    local middle1 = Joint.new(
        Vector3.new(1, 0, 0),
        Vector3.new(0, 1, 0),  -- Y-axis rotation
        -math.pi/3,            -- -60 degrees
        math.pi/3              -- 60 degrees
    )

    -- Middle joint (Y-axis rotation) - only allows rotation in XZ plane
    local middle2 = Joint.new(
        Vector3.new(2, 0, 0),
        Vector3.new(0, 1, 0),  -- Y-axis rotation
        -math.pi/3,            -- -60 degrees
        math.pi/3              -- 60 degrees
    )
    
    -- End joint (X-axis rotation) - different rotation axis for testing
    local end_joint = Joint.new(
        Vector3.new(3, 0, 0),
        Vector3.new(1, 0, 0),  -- X-axis rotation for variety
        -math.pi/2,            -- -90 degrees
        math.pi/2              -- 90 degrees
    )
    
    -- Add joints to solver
    solver:addJoint(root)
    solver:addJoint(middle1)
    solver:addJoint(middle2)
    solver:addJoint(end_joint)
    
    return solver
end

-- Demonstration function
local function demonstrateFABRIK()
    local solver = createExample()
    
    -- Target position
    local target = Vector3.new(2.54126, 0.78234, 0.5)
    
    -- Print initial positions
    print("Initial positions:")
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d: (%.2f, %.2f, %.2f)", 
            i, joint.position.x, joint.position.y, joint.position.z))
        print(string.format("  Local rotation axis: (%.2f, %.2f, %.2f)",
            joint.localAxisOfRotation.x, joint.localAxisOfRotation.y, joint.localAxisOfRotation.z))
    end
    
    -- Update world data before solving
    solver:updateWorldData()
    
    -- Solve IK
    solver:solve(target)
    
    -- Print new positions
    print("\nAfter solving for target " .. 
          string.format("(%.2f, %.2f, %.2f):", target.x, target.y, target.z))
    
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d: (%.2f, %.2f, %.2f)", 
            i, joint.position.x, joint.position.y, joint.position.z))
    end
    
    -- Print rotation axes (should now be transformed by parent rotations)
    print("\nJoint world rotation axes after solving:")
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d world axis: (%.2f, %.2f, %.2f)", 
            i, joint.worldAxisOfRotation.x, joint.worldAxisOfRotation.y, joint.worldAxisOfRotation.z))
    end
    
    -- Print joint angles
    print("\nJoint angles relative to their rotation axes:")
    for i, joint in ipairs(solver.joints) do
        print(string.format("Joint %d angle: %.2f degrees (min: %.2f, max: %.2f)", 
            i, math.deg(joint.angle), math.deg(joint.minAngle), math.deg(joint.maxAngle)))
    end
    
    -- Check if target is reachable
    local endEffector = solver.joints[#solver.joints].position
    local distToTarget = endEffector:distance(target)
    print(string.format("\nDistance from end effector to target: %.6f", distToTarget))
end

-- Run demonstration
demonstrateFABRIK()