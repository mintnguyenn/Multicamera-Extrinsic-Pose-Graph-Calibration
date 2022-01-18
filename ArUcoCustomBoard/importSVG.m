function img = importSVG(imgName)
% Load SVG image and convert to grayscale

% Import the file
imgColour = importdata(imgName);
img = rgb2gray(imgColour);

end
