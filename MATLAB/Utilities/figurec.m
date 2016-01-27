% FIGUREC - create a figure window in a non-overlapping (cascading)
%           location
% 
% USAGE:
% 
% figurec
% figurec(...)
% h=figurec
% h=figurec(...)
% 
% FIGUREC acts just like the Matlab FIGURE command, with all arguments
% passed through, except that the new figure is created a little to the
% right and down from the highest numbered figure currently existing, so
% that they won't overlap. If moving the location would push the figure too
% close to the edge of the screen, then the new figure is created in the
% default location as usual. (Subsequent figures will again be cascaded.)
% 
% EXAMPLE:
% 
% close all;for n=1:20;figurec('color',rand(1,3));plot(0,0);title('Sample');end

function varargout=figurec(varargin)
f = findobj(0,'type','figure'); % list of existing figures
ss=get(0,'ScreenSize'); % pixel size of entire screen
h=figure(varargin{:}); % create figure using pass-through arguments
hp = get(h,'pos'); % size of new figure when created
if ~isempty(f)
    f=max(f);
    u=get(f,'units');
    set(f,'units','pixels')
    p=get(max(f),'pos');
    set(f,'units',u)
    % if moving won't push too far, move; else leave in default location
    if p(1)+50+hp(3) <= ss(3) & p(2) >= 5
        u=get(h,'units');
        ss = get(0,'screensize');
        set(h,'units','pixels')
        set(h,'pos',[p(1)+50 p(2)-50 hp(3:4)]);
        set(h,'units',u)
    end
end
if nargout > 0
    varargout{1}=h;
end
