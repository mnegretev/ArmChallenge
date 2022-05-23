function obj_position = get_nearest_object(xyz, rgb)
    % REQUIRES STATISTICS AND MACHINE LEARNING TOOLBOX
    close all
    figure
    imshow(reshape(rgb, 480, 270, 3));
    title("Original Image")
    
    %%% Remove table from rgb image
    for i=1:length(rgb)
        if xyz(i,3) < 0.02
            rgb(i,:) = [0,0,0];
        end
    end
    
    %%% Color regmentation of red, green, blue and yellow
    hsv   = rgb2hsv(rgb);
    bin_r =         hsv(:,1) >=   0/180 & hsv(:,1) <=  10/180;
    bin_r = bin_r & hsv(:,2) >= 200/255 & hsv(:,2) <= 255/255;
    bin_r = bin_r & hsv(:,3) >=  30/255 & hsv(:,3) <= 255/255;
    bin_g =         hsv(:,1) >=  50/180 & hsv(:,1) <=  70/180;
    bin_g = bin_g & hsv(:,2) >= 200/255 & hsv(:,2) <= 255/255;
    bin_g = bin_g & hsv(:,3) >=  30/255 & hsv(:,3) <= 255/255;
    bin_b =         hsv(:,1) >= 110/180 & hsv(:,1) <= 130/180;
    bin_b = bin_b & hsv(:,2) >= 200/255 & hsv(:,2) <= 255/255;
    bin_b = bin_b & hsv(:,3) >=  30/255 & hsv(:,3) <= 255/255;
    bin_y =         hsv(:,1) >=  25/180 & hsv(:,1) <=  35/180;
    bin_y = bin_y & hsv(:,2) >= 200/255 & hsv(:,2) <= 255/255;
    bin_y = bin_y & hsv(:,3) >=  30/255 & hsv(:,3) <= 255/255;
    bin = bin_r | bin_g | bin_b | bin_y;
    figure
    imshow(reshape(rgb, 480, 270, 3));
    title("Removing table")
    figure
    imshow(reshape(bin, 480, 270));
    title("Segmentation by color")
    
    %%%Remove table from xyz cloud
    counter = 1;
    for i=1:length(bin)
        if bin(i)
            filtered_cloud(counter,:) = xyz(i,:);
            counter = counter + 1;
        end
    end
    
    %%% Clusterizing with K-Means
    [idx, centroids] = kmeans(filtered_cloud, 5);
    figure
    scatter3(xyz(:,1), xyz(:,2), xyz(:,3),1, rgb)
    title('Cloud with no table')
    hold on
    scatter3(centroids(:,1), centroids(:,2), centroids(:,3), 200, "red", 'filled', 'o');
    
    %%% Finding centroid with the highest 'z'
    max_z = -1;
    obj_position = [0,0,0];
    for i=1:length(centroids)
        if centroids(i,3) > max_z
            max_z = centroids(i,3);
            obj_position = centroids(i,:);
        end
    end
            