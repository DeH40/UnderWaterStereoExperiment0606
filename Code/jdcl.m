
%使用前请先使用Matlab双目标定工具箱导入双目标定参数到工作区

stereoParams = stereoParamsVar;

% 输入要进行精度测量的图片
I1 = imread('D:\20200606\100\1\image_0\Explorer_HD1080_SN24088_17-13-47.png');
I2 = imread('D:\20200606\100\1\image_1\Explorer_HD1080_SN24088_17-13-47.png');

% You can use the calibration data to rectify stereo images.
% 修正图像，对极约束 需要标定参数stereoParams
%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
I1 = undistortImage(I1,stereoParams.CameraParameters1);
I2 = undistortImage(I2,stereoParams.CameraParameters2);


% Detect checkerboards in images
%检测图片中的棋盘格角点
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(I1, I2);

%左图checkpoints坐标
checkpoints1 = imagePoints(:,:,1,1); 
%右图checkpoints坐标
checkpoints2 = imagePoints(:,:,1,2); 

%直接三角化得到三维坐标
tri = triangulate(checkpoints1,checkpoints2,stereoParams);

[r,c] = size(tri);
tempPoint = tri(1,:);
depth = zeros(r,1);
%depth = norm(tri);
%用于保存两点距离的矩阵
distance = zeros(r,1);
%计算相邻角点的距离
for i = 2:r
    %norm(tempPoint,tri(i,:));
    depth(i,1) = norm(tri(i,:));
    d = sqrt((tempPoint(1) - tri(i,1))^2 +  (tempPoint(2) - tri(i,2))^2+(tempPoint(3) - tri(i,3))^2);
    distance(i,:) = d ;
    tempPoint = tri(i,:);
end    

%disp = checkpoints1 - checkpoints2;
%b = stereoParams.TranslationOfCamera2(1);
%f = stereoParams.CameraParameters1.FocalLength(2);
%[r,c] = size(disp);
%z = zeros(c,1);
%for i = 1:r
%   z(i,1) = b*f/disp(i,1);
%end

%xx = disp(2,1);