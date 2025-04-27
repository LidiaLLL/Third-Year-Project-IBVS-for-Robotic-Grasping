img1 = imread('C:\Users\Lidia\OneDrive\Desktop\new cube screenshots\cube_distance_1.png');
img2 = imread('C:\Users\Lidia\OneDrive\Desktop\new cube screenshots\cube_distance_2.png');

gray1 = rgb2gray(img1);
gray2 = rgb2gray(img2);

points1 = detectSIFTFeatures(gray1);
points2 = detectSIFTFeatures(gray2);

chosenpoints1 = points1([1,2,3,4]);

[features1, validPoints1] = extractFeatures(gray1, chosenpoints1);
[features2, validPoints2] = extractFeatures(gray2, points2);

indexPairs = matchFeatures(features1, features2, ...
    'MatchThreshold', 100, 'MaxRatio', 0.8);

matchedPoints1 = validPoints1(indexPairs(:,1));
matchedPoints2 = validPoints2(indexPairs(:,2));


figure;
showMatchedFeatures(gray1, gray2, matchedPoints1, matchedPoints2, ...
    'montage');
title('Selected SIFT Features and Their Matches');
