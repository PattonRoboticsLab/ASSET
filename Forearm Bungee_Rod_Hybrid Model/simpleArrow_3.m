% simpleArrow:  draw arrow in 3D
%************** MATLAB "M" function  *************
% SYNTAX:     lineSeq=simpleArrow(aStart,aEnd,aColor,linWid)
% INPUTS:     aStart      1 by 2 startpoint
%             aEnd        1 by 2 endpoint
%             aColor      (optional) color spec (enter zero for no plot)
%             lineWid     (optional)  spec  -  thickness of lines
% REVISIONS:  2/8/2000    (patton) INITIATED
%             2018-Jan-14 (patton) fixed comments, added taper
%             2023-Nov-6  (wilson) adjusted for 3D coordinates
%~~~~~~~~~~~~~~~~~~~~~ Begin : ~~~~~~~~~~~~~~~~~~~~~~~~

function lineSeq = simpleArrow_3(aStart, aEnd, aColor, linWid)
    if ~exist('aColor','var') || isempty(aColor), aColor='r'; end; % if not passed
    if ~exist('linWid','var') || isempty(linWid), linWid=3; end;  % if not passed

    q = 0.15;      % length of arrowhead as a fraction of arrow length
    c = 0.1;       % width of arrowhead as a fraction of arrow length

    v = aEnd - aStart; 
    mag = norm(v); 
    if mag == 0, return; end   % do nothing if no length
    v1 = v / mag;              % unit vector - normalize
    if all(v1(1:2) == 0)
        p1 = [1 0 0];          % Handle the case where the arrow is vertical
    else
        p1 = cross(v1, [0 0 1]); % Find a vector perpendicular to v1 in the x-y plane
    end
    p1 = p1 / norm(p1);        % normalize

    % points for the arrowhead
    aBreak = aStart + (1 - q) * mag * v1;    % arrowhead base point
    aBreakPlus = aBreak + 0.04 * mag * v1;   % slightly beyond base for taper effect
    Lside = aBreak + 0.5 * c * mag * p1;     % left side of arrowhead
    Rside = aBreak - 0.5 * c * mag * p1;     % right side of arrowhead

    % assemble line Sequence for the shaft and the sides of the arrowhead
    lineSeq = [aStart; aBreakPlus; Lside; aEnd; Rside; aBreakPlus];
    
    % draw the arrow shaft
    plot3([aStart(1), aBreak(1)], [aStart(2), aBreak(2)], [aStart(3), aBreak(3)], ...
          'Color', aColor, 'LineWidth', linWid);
    hold on;
    
    % draw the arrowhead as a filled polygon
    fill3([Lside(1), aEnd(1), Rside(1)], [Lside(2), aEnd(2), Rside(2)], ...
          [Lside(3), aEnd(3), Rside(3)], aColor);
    
    hold off;
end