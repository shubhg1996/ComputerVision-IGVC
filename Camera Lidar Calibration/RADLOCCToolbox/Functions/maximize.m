function maximize(h)

% MAXIMIZE   maximize figure windows
% ====================================================================
%
%        Berne University of Applied Sciences
%
%        School of Engineering and Information Technology
%        Division of Electrical- and Communication Engineering
%
% ====================================================================
%                       maximize figure windows
% ====================================================================
%
% Author:    Alain Trostel
% e-mail:    alain.trostel@bfh.ch
% Date:      June 2007
% Version:   4.1
%
% ====================================================================
%
% function maximize(h)
%
% Input parameters
% -----------------
%   h             handle(s) of the figure window
%
%
% Output parameters
% ------------------
%   The function has no output parameters.
%
%
% Used files
% -----------
%   - windowMaximize.dll
%
%
% Examples
% ---------
%   % maximize the current figure
%   ------------------------------
%   maximize;
%
%
%   % maximize the current figure
%   ------------------------------
%   maximize(gcf);
%
%
%   % maximize the specified figure
%   --------------------------------
%   h = figure;
%   maximize(h);
%
%
%   % maximize the application window
%   ----------------------------------
%   maximize(0);
%
%
%   % maximize more than one figure
%   --------------------------------
%   h(1) = figure;
%   h(2) = figure;
%   maximize(h);
%
%
%   % maximize all figures
%   -----------------------
%   maximize('all');
%
%
%   % maximize a GUI in the OpeningFcn
%   -----------------------------------
%
%   % --- Executes just before untitled is made visible.
%   function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
%   % This function has no output args, see OutputFcn.
%   % hObject    handle to figure
%   % eventdata  reserved - to be defined in a future version of MATLAB
%   % handles    structure with handles and user data (see GUIDATA)
%   % varargin   command line arguments to untitled (see VARARGIN)
%
%   % Choose default command line output for untitled
%   handles.output = hObject;
%
%   % Update handles structure
%   guidata(hObject, handles);
%
%   % UIWAIT makes untitled wait for user response (see UIRESUME)
%   % uiwait(handles.figure1);
%
%   % maximize the GUI
%   set(hObject,'Visible','on');
%   maximize(hObject);

pause(0.00001);
frame_h = get(handle(h),'JavaFrame');
set(frame_h,'Maximized',1);
end