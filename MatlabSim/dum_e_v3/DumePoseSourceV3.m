classdef DumePoseSourceV3 < matlab.System
    properties (Nontunable)
        StateFile = ''
        SampleTime = 0.01
        MoveDuration = 0
        GripperClosedMeters = -0.009
        GripperOpenMeters = 0.010
    end

    properties (Access = private)
        StartPose
        StartVelocity
        StartAcceleration
        TargetPose
        StartTime
        C0
        C1
        C2
        C3
        C4
        C5
    end

    methods (Access = protected)
        function setupImpl(obj)
            if obj.SampleTime <= 0 || obj.MoveDuration < 0
                error('DumePoseSourceV3:InvalidTiming', ...
                    'SampleTime must be positive and MoveDuration nonnegative.');
            end

            obj.TargetPose = read_dume_pose_v3( ...
                obj.StateFile, obj.GripperClosedMeters, obj.GripperOpenMeters);
            obj.StartPose = obj.TargetPose;
            obj.StartVelocity = zeros(6,1);
            obj.StartAcceleration = zeros(6,1);
            obj.StartTime = 0;
            updateCoefficients(obj);
        end

        function [position, velocity, acceleration] = stepImpl(obj, simTime)
            [position, velocity, acceleration] = evaluateTrajectory(obj, simTime);
            newTarget = read_dume_pose_v3( ...
                obj.StateFile, obj.GripperClosedMeters, obj.GripperOpenMeters);
            if any(abs(newTarget - obj.TargetPose) > 1e-10)
                obj.StartPose = position;
                obj.StartVelocity = velocity;
                obj.StartAcceleration = acceleration;
                obj.TargetPose = newTarget;
                obj.StartTime = simTime;
                updateCoefficients(obj);
                [position, velocity, acceleration] = evaluateTrajectory(obj, simTime);
            end
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, ...
                'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function [size1, size2, size3] = getOutputSizeImpl(~)
            size1 = [6 1];
            size2 = [6 1];
            size3 = [6 1];
        end

        function [type1, type2, type3] = getOutputDataTypeImpl(~)
            type1 = 'double';
            type2 = 'double';
            type3 = 'double';
        end

        function [complex1, complex2, complex3] = isOutputComplexImpl(~)
            complex1 = false;
            complex2 = false;
            complex3 = false;
        end

        function [fixed1, fixed2, fixed3] = isOutputFixedSizeImpl(~)
            fixed1 = true;
            fixed2 = true;
            fixed3 = true;
        end
    end

    methods (Access = private)
        function updateCoefficients(obj)
            duration = obj.MoveDuration;
            displacement = obj.TargetPose - obj.StartPose;
            obj.C0 = obj.StartPose;
            obj.C1 = duration*obj.StartVelocity;
            obj.C2 = 0.5*duration^2*obj.StartAcceleration;
            obj.C3 = 10*displacement - 6*obj.C1 - 1.5*duration^2*obj.StartAcceleration;
            obj.C4 = -15*displacement + 8*obj.C1 + 1.5*duration^2*obj.StartAcceleration;
            obj.C5 = 6*displacement - 3*obj.C1 - 0.5*duration^2*obj.StartAcceleration;
        end

        function [position, velocity, acceleration] = evaluateTrajectory(obj, simTime)
            duration = obj.MoveDuration;
            elapsed = simTime - obj.StartTime;
            if duration == 0 || elapsed >= duration
                position = obj.TargetPose;
                velocity = zeros(6,1);
                acceleration = zeros(6,1);
                return;
            end

            u = max(0, elapsed/duration);
            position = obj.C0 + obj.C1*u + obj.C2*u^2 + obj.C3*u^3 + ...
                obj.C4*u^4 + obj.C5*u^5;
            velocity = (obj.C1 + 2*obj.C2*u + 3*obj.C3*u^2 + ...
                4*obj.C4*u^3 + 5*obj.C5*u^4)/duration;
            acceleration = (2*obj.C2 + 6*obj.C3*u + 12*obj.C4*u^2 + ...
                20*obj.C5*u^3)/duration^2;
        end
    end
end
