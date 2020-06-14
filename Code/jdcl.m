
%ʹ��ǰ����ʹ��Matlab˫Ŀ�궨�����䵼��˫Ŀ�궨������������

stereoParams = stereoParamsVar;

% ����Ҫ���о��Ȳ�����ͼƬ
I1 = imread('D:\20200606\100\1\image_0\Explorer_HD1080_SN24088_17-13-47.png');
I2 = imread('D:\20200606\100\1\image_1\Explorer_HD1080_SN24088_17-13-47.png');

% You can use the calibration data to rectify stereo images.
% ����ͼ�񣬶Լ�Լ�� ��Ҫ�궨����stereoParams
%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
I1 = undistortImage(I1,stereoParams.CameraParameters1);
I2 = undistortImage(I2,stereoParams.CameraParameters2);


% Detect checkerboards in images
%���ͼƬ�е����̸�ǵ�
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(I1, I2);

%��ͼcheckpoints����
checkpoints1 = imagePoints(:,:,1,1); 
%��ͼcheckpoints����
checkpoints2 = imagePoints(:,:,1,2); 

%ֱ�����ǻ��õ���ά����
tri = triangulate(checkpoints1,checkpoints2,stereoParams);

[r,c] = size(tri);
tempPoint = tri(1,:);
depth = zeros(r,1);
%depth = norm(tri);
%���ڱ����������ľ���
distance = zeros(r,1);
%�������ڽǵ�ľ���
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