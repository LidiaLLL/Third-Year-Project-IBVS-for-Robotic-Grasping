img = imread("C:\Users\Lidia\OneDrive\Desktop\new cube screenshots\cube_distance_1.png");

time = tic;

grayImg = rgb2gray(img);

points = detectSIFTFeatures(grayImg);
imshow(grayImg); hold on;
plot(points.selectStrongest(4)); % adjust number if needed

% Color thresholding to find the blue cube face
blueMask = img(:,:,3) > 100 & img(:,:,1) < 50 & img(:,:,2) < 50;

% Get positions of keypoints
locations = round(points.Location);

% Check each keypoint for being in a blue region and dark spot
isDot = false(points.Count,1);
for i = 1:points.Count
    x = locations(i,1);
    y = locations(i,2);
    if x > 0 && y > 0 && x <= size(img,2) && y <= size(img,1)
        if blueMask(y,x) %&& grayImg(y,x) < 50  % dark point in blue area
            isDot(i) = true;
        end
    end
end

% Keep only likely black dot keypoints
dotPoints = points(isDot);

% Suppose dotPoints contains your filtered keypoints
if dotPoints.Count >= 4
    selectedPoints = dotPoints.selectStrongest(4);  % or use indexing
else
    warning('Less than 4 points detected!');
    selectedPoints = dotPoints;
end

%imshow(img); hold on;
%plot(dotPoints);
%title('Detected black dots on blue cube face');
