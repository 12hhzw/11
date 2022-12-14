function varargout = diposeImage(varargin)
% DIPOSEIMAGE MATLAB code for diposeImage.fig
%      DIPOSEIMAGE, by itself, creates a new DIPOSEIMAGE or raises the existing
%      singleton*.
%
%      H = DIPOSEIMAGE returns the handle to a new DIPOSEIMAGE or the handle to
%      the existing singleton*.
%
%      DIPOSEIMAGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIPOSEIMAGE.M with the given input arguments.
%
%      DIPOSEIMAGE('Property','Value',...) creates a new DIPOSEIMAGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before diposeImage_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to diposeImage_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help diposeImage

% Last Modified by GUIDE v2.5 26-Dec-2019 00:33:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @diposeImage_OpeningFcn, ...
                   'gui_OutputFcn',  @diposeImage_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before diposeImage is made visible.
function diposeImage_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to diposeImage (see VARARGIN)

% Choose default command line output for diposeImage
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes diposeImage wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = diposeImage_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%??????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
I = rgb2gray(handles.image);%????????????
imhist(I);%??????????


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
I = rgb2gray(handles.image);
I = histeq(I);%????????????
imhist(I);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????????
[fp,pn,fg]=uigetfile('*.*','????????');
I = imread([pn fp]);
axes(handles.axes1);%??????????????????
imshow(I);
handles.image = I;
guidata(hObject,handles)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????????????????????
image1 = handles.image;
figure('NumberTitle', 'off', 'Name', '??????????'); 
subplot(2,2,1);
imshow(image1);
title('????????');

X = 0.7;
image = im2double(image1);
image2 = (image .^ X);%????????????
subplot(2,2,2);
imshow(image2);
title(['X:',num2str(X)]);

X = 1.6;
image = im2double(image1);
image2 = (image .^ X);%????????????
subplot(2,2,3);
imshow(image2);
title(['X:',num2str(X)]);
 

X = 3;
image = im2double(image1);
image2 = (image .^ X);%????????????
subplot(2,2,4);
imshow(image2);
title(['X:',num2str(X)]);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????????????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
I = handles.image;%????????????
J = imrotate(I,-45,'crop');%????????
imshow(J)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????????????????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
I = handles.image;
J = flipdim(I,2);%????????????????
imshow(J);



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double

%????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
number = str2double(get(hObject,'String'));%????????????????
I = handles.image;
J = imnoise(I,'salt & pepper',number);%????????????
imshow(J);


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double

%????????????
set(handles.axes2,'HandleVisibility','ON');
axes(handles.axes2);
number = str2double(get(hObject,'String'));
I = handles.image;
J = imnoise(I,'gaussian',number);%????????????
imshow(J);



% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double

%????????????????????????
number= str2double(get(hObject,'String'));
I = rgb2gray(handles.image);
J = imnoise(I,'salt & pepper' ,number);%????????????
K1 = filter2(fspecial('average',3),J)/255;%??????????
K2 = medfilt2(J,[3,3]);%??????????
K3 = wiener2(J,[5,5]);%????????????
figure('NumberTitle', 'off', 'Name', '????????????????????'); 
subplot(2,2,1);
imshow(J);
title('????????????????');
subplot(2,2,2);
imshow(K1);
title('??????????');
subplot(2,2,3);
imshow(K2);
title('??????????');
subplot(2,2,4);
imshow(K3);
title('????????????');


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double

%????????????????????????
number = str2double(get(hObject,'String'));
I1 = rgb2gray(handles.image);
figure('NumberTitle', 'off', 'Name', '????????????????????');
subplot(2,2,1);
imshow(I1);title('????????');
I2 = imnoise(I1,'salt & pepper' ,number);%????????????
subplot(2,2,2);
imshow(I2);title('????????????????');
f = double(I2);
g = fft2(f);
g = fftshift(g);
[N1,N2] = size(g);
n = 2;
d0 = 50;
n1 = fix(N1/2);
n2 = fix(N2/2);
for i = 1:N1
    for j = 1:N2
        d = sqrt((i - n1)^2 + (j - n2)^2);
        % Buttetworth????????
        h = 1/(1 + (d/d0)^(2*n));
        result1(i,j) = h*g(i,j);
        % ????????????
        if d>30
            result2(i,j) = 0;
        else
            result2(i,j) = g(i,j);
        end
    end
end
result1 = ifftshift(result1);
result2 = ifftshift(result2);
X2 = ifft2(result1);
X3 = uint8(real(X2));
subplot(2,2,3);
imshow(X3,[]);
title('Buttetworth????????');
X4 = ifft2(result2);
X5 = uint8(real(X4));
subplot(2,2,4);
imshow(X5,[]);
title('????????????');




% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%????roberts??prewitt??sobel????????????????????????????????
I=rgb2gray(handles.image);
BW1 = edge(I,'Roberts');%Roberts????
BW2 = edge(I,'Prewitt');%Prewitt????
BW3 = edge(I,'Sobel');%Sobel????
[M,N] = size(I);%????????????
BW4 = zeros(size(I));
for x = 2:M-1
    for y = 2:N-1
        BW4(x,y) = I(x+1,y) + I(x-1,y) + I(x,y+1) + I(x,y-1) - 4*I(x,y);
    end;
end

figure('NumberTitle', 'off', 'Name', '????roberts??prewitt??sobel????????????????????????????????');
subplot(2,2,1);
imshow(BW1);title('Roberts????????????');
subplot(2,2,2);
imshow(BW2);title('Prewitt????????????');
subplot(2,2,3);
imshow(BW3);title('Sobel????????????');
subplot(2,2,4);
imshow(BW4);title('????????????????????');
