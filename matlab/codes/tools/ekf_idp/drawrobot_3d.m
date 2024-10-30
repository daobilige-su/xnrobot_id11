%DRAWROBOT Draw robot.
%   DRAWROBOT(X,COLOR) draws a robot at pose X = [x y theta] such
%   that the robot reference frame is attached to the center of
%   the wheelbase with the x-axis looking forward. COLOR is a
%   [r g b]-vector or a color string such as 'r' or 'g'.
%
%   DRAWROBOT(X,COLOR,TYPE) draws a robot of type TYPE. Five
%   different models are implemented:
%      TYPE = 0 draws only a cross with orientation theta
%      TYPE = 1 is a differential drive robot without contour
%      TYPE = 2 is a differential drive robot with round shape
%      TYPE = 3 is a round shaped robot with a line at theta
%      TYPE = 4 is a differential drive robot with rectangular shape
%      TYPE = 5 is a rectangular shaped robot with a line at theta
%
%   DRAWROBOT(X,COLOR,TYPE,W,L) draws a robot of type TYPE with
%   width W and length L in [m].
%
%   H = DRAWROBOT(...) returns a column vector of handles to all
%   graphic objects of the robot drawing. Remember that not all
%   graphic properties apply to all types of graphic objects. Use
%   FINDOBJ to find and access the individual objects.
%
%   See also DRAWRECT, DRAWARROW, FINDOBJ, PLOT.

% v.1.0, 16.06.03, Kai Arras, ASL-EPFL
% v.1.1, 12.10.03, Kai Arras, ASL-EPFL: uses drawrect
% v.1.2, 03.12.03, Kai Arras, CAS-KTH : types implemented


function drawrobot_3d(mu,color)

r = 0.3;

drawprobellipse_3d(mu(1:3),r*eye(3), 0.6, color);

end
