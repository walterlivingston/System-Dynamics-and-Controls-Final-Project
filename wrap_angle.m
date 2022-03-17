%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%
%   function angle=wrap_angle(angle,[low high])
%
%   Written by the GPS and Vehicle Dynamics Lab (GAVLAB)
%   at Auburn University
% 
%   function to wrap an angle between low and high range
%   the function can be used in degrees or radians
%
%   Examples:
%
%       angle=wrap(630,[0 360] returns 270
%       angle=wrap(630,[-180 180]) returns -90
%       angle=wrap(11,[0 2*pi]) returns 4.7168
%       angle=wrap(11,[-pi pi]) returns -1.5664
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function angle=wrap_angle(angle,range)

low=range(1);
high=range(2);

ln=length(angle);
diff=high-low;

for k=1:ln
    while(angle(k)>high  | angle(k)<low)
        if(angle(k)>high)
            angle(k:ln)=angle(k:ln)-diff;
        else
            angle(k:ln)=angle(k:ln)+diff;
        end
    end
end